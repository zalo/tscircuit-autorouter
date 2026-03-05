import type { GraphicsObject, Line } from "graphics-debug"
import type { Point } from "polyanya"
import { rectToPolygon, distance } from "polyanya"
import { BaseSolver } from "../BaseSolver"
import type { SimpleRouteJson, ConnectionPoint } from "../../types"
import { getConnectionPointLayers } from "../../utils/connection-point-utils"
import { mergeOverlappingRects } from "../PolyanyaSolver/mergeOverlappingRects"
import type { ResolvedPath } from "./types"
import { getColorMap } from "../colors"
import { getConnectivityMapFromSimpleRouteJson } from "lib/utils/getConnectivityMapFromSimpleRouteJson"
import {
  buildRawCdt,
  type RawCdt,
  type CdtEdge,
  type CdtTriangle,
} from "./TopologicalCdt"

/**
 * Topological Rubberband Autorouter (gEDA-style)
 *
 * Based on the gEDA toporouter by Anthony Blake (2009), implementing
 * the SURF topological routing approach:
 *
 * 1. BUILD CDT: Constrained Delaunay Triangulation from SRJ obstacles.
 *    The CDT is built ONCE per layer and never rebuilt.
 *
 * 2. ROUTE VIA A* THROUGH CDT EDGES: Each route is an A* search through
 *    CDT triangles. Candidate waypoints are placed ON CDT edges (not in
 *    free space). Each edge stores an ordered list of route crossings.
 *    The crossing order defines the topological embedding.
 *
 * 3. EDGE CAPACITY: Each CDT edge has a capacity (its geometric length)
 *    and flow (space consumed by existing route crossings). New routes
 *    can only cross edges with remaining capacity.
 *
 * 4. RUBBER-BANDING: After all routes are embedded, each path is pulled
 *    tight by computing tangent arcs around obstacle vertices while
 *    preserving the crossing order on shared edges.
 *
 * 5. SPACING: Route vertices on shared edges are spread via force-based
 *    relaxation to maintain minimum clearances.
 */
export class TopologicalPathSolver extends BaseSolver {
  override getSolverName(): string {
    return "TopologicalPathSolver"
  }

  private srj: SimpleRouteJson
  private colorMap: Record<string, string>
  private margin: number
  private minTraceWidth: number
  private viaDiameter: number
  private maxLayerCount: number
  private layerCount: number

  /** Per-layer raw CDTs (built once, never rebuilt) */
  private cdts: (RawCdt | null)[] = []

  /** Per-layer base obstacle polygons */
  private baseObstaclePolygons: Point[][][] = []

  /** All connections to route */
  private connections: Array<{
    name: string
    originalStart: Point
    originalEnd: Point
    start: Point
    end: Point
    startLayerZ: number
    endLayerZ: number
  }> = []

  /** Routed paths */
  private routes: TopoRouteState[] = []

  /** Final output */
  private resolvedPaths: ResolvedPath[] = []

  /** Solver phase */
  private phase:
    | "build-cdt"
    | "route"
    | "rubberband"
    | "commit"
    | "done" = "build-cdt"
  private routeIndex = 0
  private rubberBandIter = 0

  /** Layer name → z-index */
  private layerNameToZ = new Map<string, number>()

  /** Validation result */
  validationResult: {
    totalConnections: number
    routedConnections: number
    unroutedConnections: string[]
    crossNetCrossings: Array<{ netA: string; netB: string }>
  } | null = null

  constructor(params: {
    srj: SimpleRouteJson
    colorMap?: Record<string, string>
    minTraceWidth?: number
    margin?: number
  }) {
    super()
    this.MAX_ITERATIONS = 100_000_000
    this.srj = params.srj
    this.minTraceWidth = params.minTraceWidth ?? params.srj.minTraceWidth
    this.margin =
      params.margin ?? params.srj.defaultObstacleMargin ?? this.minTraceWidth
    this.viaDiameter = params.srj.minViaDiameter ?? 0.6
    this.maxLayerCount = Math.max(1, params.srj.layerCount ?? 2)
    this.layerCount = this.maxLayerCount

    const connMap = getConnectivityMapFromSimpleRouteJson(params.srj)
    this.colorMap = params.colorMap ?? getColorMap(params.srj, connMap)

    // Layer name → z-index
    const allLayerNames = this.getAllLayerNames()
    for (let z = 0; z < allLayerNames.length; z++) {
      this.layerNameToZ.set(allLayerNames[z]!, z)
    }

    // Base obstacle polygons per layer
    this.baseObstaclePolygons = []
    for (let z = 0; z < this.maxLayerCount; z++) {
      const layerName = allLayerNames[z]!
      const layerPolys: Point[][] = []
      for (const obs of params.srj.obstacles) {
        if (!obs.layers.includes(layerName)) continue
        layerPolys.push(
          rectToPolygon(
            obs.center.x,
            obs.center.y,
            obs.width,
            obs.height,
            this.margin,
          ),
        )
      }
      this.baseObstaclePolygons.push(layerPolys)
    }

    // Build connections with nudged endpoints
    this.connections = params.srj.connections.map((conn) => {
      const pts = conn.pointsToConnect
      const originalStart = { x: pts[0]!.x, y: pts[0]!.y }
      const originalEnd = {
        x: pts[pts.length - 1]!.x,
        y: pts[pts.length - 1]!.y,
      }

      const connNames = [conn.name]
      if (conn.rootConnectionName && conn.rootConnectionName !== conn.name) {
        connNames.push(conn.rootConnectionName)
      }
      if (conn.name.includes("__")) {
        for (const part of conn.name.split("__")) {
          if (!connNames.includes(part)) connNames.push(part)
        }
      }

      const nudgedStart = this.nudgeOutOfObstacle(
        originalStart,
        originalEnd,
        params.srj.obstacles,
        connNames,
      )
      const nudgedEnd = this.nudgeOutOfObstacle(
        originalEnd,
        originalStart,
        params.srj.obstacles,
        connNames,
      )

      return {
        name: conn.name,
        originalStart,
        originalEnd,
        start: nudgedStart,
        end: nudgedEnd,
        startLayerZ: this.connectionPointToLayerZ(pts[0]!),
        endLayerZ: this.connectionPointToLayerZ(pts[pts.length - 1]!),
      }
    })

    // Sort: shortest first
    this.connections.sort(
      (a, b) => distance(a.start, a.end) - distance(b.start, b.end),
    )
  }

  private getAllLayerNames(): string[] {
    const layerSet = new Set<string>()
    for (const obs of this.srj.obstacles) {
      for (const l of obs.layers) layerSet.add(l)
    }
    for (const conn of this.srj.connections) {
      for (const pt of conn.pointsToConnect) {
        if ("layer" in pt) layerSet.add(pt.layer)
        if ("layers" in pt) {
          for (const l of pt.layers) layerSet.add(l)
        }
      }
    }
    const layers = Array.from(layerSet)
    layers.sort((a, b) => {
      if (a === "top") return -1
      if (b === "top") return 1
      if (a === "bottom") return 1
      if (b === "bottom") return -1
      return a.localeCompare(b)
    })
    return layers.length > 0 ? layers : ["top", "bottom"]
  }

  private connectionPointToLayerZ(pt: ConnectionPoint): number {
    const layers = getConnectionPointLayers(pt)
    for (const name of layers) {
      const z = this.layerNameToZ.get(name)
      if (z !== undefined) return z
    }
    return 0
  }

  private nudgeOutOfObstacle(
    pt: Point,
    other: Point,
    obstacles: SimpleRouteJson["obstacles"],
    connNames: string[],
  ): Point {
    const connected = obstacles.filter((obs) =>
      connNames.some((n) => obs.connectedTo.includes(n)),
    )
    if (connected.length === 0) return pt

    let containingObs: (typeof connected)[0] | null = null
    for (const obs of connected) {
      const halfW = obs.width / 2 + this.margin + 0.05
      const halfH = obs.height / 2 + this.margin + 0.05
      if (
        Math.abs(pt.x - obs.center.x) < halfW &&
        Math.abs(pt.y - obs.center.y) < halfH
      ) {
        if (
          !containingObs ||
          obs.width * obs.height < containingObs.width * containingObs.height
        ) {
          containingObs = obs
        }
      }
    }
    if (!containingObs) return pt

    const obs = containingObs
    const halfW = obs.width / 2 + this.margin + 0.05
    const halfH = obs.height / 2 + this.margin + 0.05
    const toOtherX = other.x - pt.x
    const toOtherY = other.y - pt.y
    const dx = pt.x - obs.center.x
    const dy = pt.y - obs.center.y

    const candidates = [
      { x: obs.center.x + halfW, y: pt.y, score: toOtherX, dist: halfW - dx },
      { x: obs.center.x - halfW, y: pt.y, score: -toOtherX, dist: halfW + dx },
      { x: pt.x, y: obs.center.y + halfH, score: toOtherY, dist: halfH - dy },
      { x: pt.x, y: obs.center.y - halfH, score: -toOtherY, dist: halfH + dy },
    ]
    candidates.sort((a, b) => {
      const aA = a.score > 0 ? 1 : 0
      const bA = b.score > 0 ? 1 : 0
      if (aA !== bA) return bA - aA
      return a.dist - b.dist
    })

    let result = { x: candidates[0]!.x, y: candidates[0]!.y }
    for (let iter = 0; iter < 5; iter++) {
      let pushed = false
      for (const obs2 of connected) {
        const hw = obs2.width / 2 + this.margin + 0.05
        const hh = obs2.height / 2 + this.margin + 0.05
        const d2x = result.x - obs2.center.x
        const d2y = result.y - obs2.center.y
        if (Math.abs(d2x) < hw && Math.abs(d2y) < hh) {
          const dr = hw - d2x
          const dl = hw + d2x
          const dt = hh - d2y
          const db = hh + d2y
          const m = Math.min(dr, dl, dt, db)
          if (m === dr) result = { x: obs2.center.x + hw, y: result.y }
          else if (m === dl) result = { x: obs2.center.x - hw, y: result.y }
          else if (m === db) result = { x: result.x, y: obs2.center.y - hh }
          else result = { x: result.x, y: obs2.center.y + hh }
          pushed = true
        }
      }
      if (!pushed) break
    }
    return result
  }

  // ===== CDT-EDGE A* ROUTING =====

  /**
   * Locate which CDT triangle contains a point.
   */
  private locateTriangle(cdt: RawCdt, pt: Point): number {
    for (let ti = 0; ti < cdt.triangles.length; ti++) {
      const tri = cdt.triangles[ti]!
      if (tri.obstacle) continue
      if (this.pointInTriangle(cdt.pts, tri.v, pt)) return ti
    }
    // Fallback: nearest non-obstacle triangle centroid
    let bestDist = Infinity
    let bestTi = -1
    for (let ti = 0; ti < cdt.triangles.length; ti++) {
      const tri = cdt.triangles[ti]!
      if (tri.obstacle) continue
      const cx =
        (cdt.pts[tri.v[0]]!.x + cdt.pts[tri.v[1]]!.x + cdt.pts[tri.v[2]]!.x) / 3
      const cy =
        (cdt.pts[tri.v[0]]!.y + cdt.pts[tri.v[1]]!.y + cdt.pts[tri.v[2]]!.y) / 3
      const d = (pt.x - cx) ** 2 + (pt.y - cy) ** 2
      if (d < bestDist) {
        bestDist = d
        bestTi = ti
      }
    }
    return bestTi
  }

  private pointInTriangle(
    pts: Point[],
    v: [number, number, number],
    p: Point,
  ): boolean {
    const a = pts[v[0]]!
    const b = pts[v[1]]!
    const c = pts[v[2]]!
    const d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y)
    const d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y)
    const d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y)
    const hasNeg = d1 < 0 || d2 < 0 || d3 < 0
    const hasPos = d1 > 0 || d2 > 0 || d3 > 0
    return !(hasNeg && hasPos)
  }

  /**
   * Get the edge index between two vertices in the CDT.
   */
  private getEdgeIdx(cdt: RawCdt, v0: number, v1: number): number {
    const key = v0 < v1 ? `${v0},${v1}` : `${v1},${v0}`
    return cdt.edgeMap.get(key) ?? -1
  }

  /**
   * Get the edge capacity remaining after existing crossings.
   * Capacity = edge length - (existing crossings * spacing)
   */
  private edgeRemainingCapacity(edge: CdtEdge): number {
    const spacing = this.minTraceWidth + this.margin
    const used = edge.crossings.length * spacing
    return edge.length - used
  }

  /**
   * A* search through CDT triangles. Returns both the path points AND
   * the edge indices crossed, so we can properly record crossings.
   */
  private routeThroughCdt(
    cdt: RawCdt,
    start: Point,
    end: Point,
    connectionName: string,
  ): { path: Point[]; edgesCrossed: number[] } | null {
    const startTri = this.locateTriangle(cdt, start)
    const endTri = this.locateTriangle(cdt, end)
    if (startTri < 0 || endTri < 0) return null

    interface AStarNode {
      triIdx: number
      point: Point
      g: number
      f: number
      parent: AStarNode | null
      entryEdge: number // CDT edge index crossed to reach this triangle
    }

    const spacing = this.minTraceWidth + this.margin
    const baseNet = TopologicalPathSolver.baseNetName(connectionName)

    const open: AStarNode[] = []
    const closed = new Set<number>()
    const h = (p: Point) => distance(p, end)

    open.push({
      triIdx: startTri,
      point: start,
      g: 0,
      f: h(start),
      parent: null,
      entryEdge: -1,
    })

    while (open.length > 0) {
      let bestIdx = 0
      for (let i = 1; i < open.length; i++) {
        if (open[i]!.f < open[bestIdx]!.f) bestIdx = i
      }
      const current = open[bestIdx]!
      open.splice(bestIdx, 1)

      if (closed.has(current.triIdx)) continue
      closed.add(current.triIdx)

      if (current.triIdx === endTri) {
        // Reconstruct path + edge list
        const path: Point[] = [end]
        const edgesCrossed: number[] = []
        let node: AStarNode | null = current
        while (node) {
          path.push(node.point)
          if (node.entryEdge >= 0) edgesCrossed.push(node.entryEdge)
          node = node.parent
        }
        path.reverse()
        edgesCrossed.reverse()
        return { path, edgesCrossed }
      }

      const tri = cdt.triangles[current.triIdx]!
      const triEdges = [
        { neighborSlot: 0, v0: tri.v[1], v1: tri.v[2] },
        { neighborSlot: 1, v0: tri.v[2], v1: tri.v[0] },
        { neighborSlot: 2, v0: tri.v[0], v1: tri.v[1] },
      ]

      for (const { neighborSlot, v0, v1 } of triEdges) {
        const neighborTri = tri.n[neighborSlot]
        if (neighborTri < 0) continue
        if (closed.has(neighborTri)) continue

        const neighborTriObj = cdt.triangles[neighborTri]!
        if (neighborTriObj.obstacle) continue

        const edgeIdx = this.getEdgeIdx(cdt, v0, v1)
        if (edgeIdx < 0) continue

        const edge = cdt.edges[edgeIdx]!
        if (edge.isConstraint) continue

        // Check capacity
        if (this.edgeRemainingCapacity(edge) < spacing) {
          const allSameNet = edge.crossings.every(
            (c) => TopologicalPathSolver.baseNetName(c.connectionName) === baseNet,
          )
          if (!allSameNet) continue
        }

        // Crossing point: midpoint (will be adjusted by spaceEdges later)
        const p0 = cdt.pts[v0]!
        const p1 = cdt.pts[v1]!
        const crossPt = { x: (p0.x + p1.x) / 2, y: (p0.y + p1.y) / 2 }

        const stepDist = distance(current.point, crossPt)
        open.push({
          triIdx: neighborTri,
          point: crossPt,
          g: current.g + stepDist,
          f: current.g + stepDist + h(crossPt),
          parent: current,
          entryEdge: edgeIdx,
        })
      }
    }

    return null
  }

  /**
   * Record edge crossings from a routed path. Uses the known edge indices
   * from A* rather than searching by distance.
   */
  private recordEdgeCrossings(
    cdt: RawCdt,
    edgesCrossed: number[],
    connectionName: string,
    routeIdx: number,
  ) {
    for (const edgeIdx of edgesCrossed) {
      const edge = cdt.edges[edgeIdx]!
      edge.crossings.push({
        connectionName,
        t: 0.5, // initial placement at midpoint; spaceEdges will fix
        point: { x: 0, y: 0 }, // will be computed by spaceEdges
      })
    }
  }

  /**
   * gEDA-style edge spacing: distribute route crossings evenly along
   * each shared CDT edge, maintaining minimum clearance from edge
   * endpoints (obstacle vertices) and between crossings.
   *
   * After this, update all route paths to use the new crossing positions.
   */
  private spaceAllEdges(cdt: RawCdt) {
    const spacing = this.minTraceWidth + this.margin

    for (const edge of cdt.edges) {
      const n = edge.crossings.length
      if (n === 0) continue

      const ep0 = cdt.pts[edge.v0]!
      const ep1 = cdt.pts[edge.v1]!
      const edgeLen = edge.length
      if (edgeLen < 1e-9) continue

      // Reserve spacing from endpoints (obstacle vertices need clearance)
      const endClear = spacing / 2
      const usableStart = endClear / edgeLen
      const usableEnd = 1 - endClear / edgeLen

      if (usableStart >= usableEnd) {
        // Edge too short — pack at midpoint
        for (let i = 0; i < n; i++) {
          edge.crossings[i]!.t = 0.5
        }
      } else if (n === 1) {
        // Single crossing — place at center of usable range
        edge.crossings[0]!.t = (usableStart + usableEnd) / 2
      } else {
        // Multiple crossings — distribute evenly in usable range
        const step = (usableEnd - usableStart) / (n - 1)
        for (let i = 0; i < n; i++) {
          edge.crossings[i]!.t = usableStart + i * step
        }
      }

      // Update crossing point positions
      for (const crossing of edge.crossings) {
        crossing.point = {
          x: ep0.x + crossing.t * (ep1.x - ep0.x),
          y: ep0.y + crossing.t * (ep1.y - ep0.y),
        }
      }
    }
  }

  /**
   * After spaceAllEdges, rebuild each route's path using the updated
   * crossing positions from the CDT edges.
   */
  // updateRoutePathsFromCrossings removed — use rebuildPathFromCrossings per-route instead

  private pointToSegmentDist(p: Point, a: Point, b: Point): number {
    const dx = b.x - a.x
    const dy = b.y - a.y
    const len2 = dx * dx + dy * dy
    if (len2 < 1e-12) return distance(p, a)
    const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2))
    const projX = a.x + t * dx
    const projY = a.y + t * dy
    return Math.hypot(p.x - projX, p.y - projY)
  }

  /**
   * Rubber-band a route: for each crossing, slide it along its CDT edge
   * to where the straight line from prev→next would intersect that edge.
   * Modifies the edge crossing records directly (single source of truth).
   */
  private rubberBandRoute(cdt: RawCdt, route: TopoRouteState) {
    const edgesCrossed = (route as any)._edgesCrossed as number[] | undefined
    if (!edgesCrossed || edgesCrossed.length === 0) return

    const spacing = this.minTraceWidth / 2 + this.margin

    // Get current crossing positions to use as "original" reference
    const crossingPts: Point[] = edgesCrossed.map((ei) => {
      const edge = cdt.edges[ei]!
      const c = edge.crossings.find((c) => c.connectionName === route.connectionName)
      return c ? { ...c.point } : { x: 0, y: 0 }
    })

    for (let i = 0; i < edgesCrossed.length; i++) {
      // prev = start or previous crossing (use updated for forward convergence)
      const prev = i === 0
        ? route.start
        : (() => {
            const prevEdge = cdt.edges[edgesCrossed[i - 1]!]!
            const prevC = prevEdge.crossings.find((c) => c.connectionName === route.connectionName)
            return prevC ? prevC.point : route.start
          })()
      // next = next crossing or end (use original to avoid cascading)
      const next = i === edgesCrossed.length - 1
        ? route.end
        : crossingPts[i + 1]!

      const edgeIdx = edgesCrossed[i]!
      const edge = cdt.edges[edgeIdx]!
      const ep0 = cdt.pts[edge.v0]!
      const ep1 = cdt.pts[edge.v1]!

      const ideal = this.bestEdgeCrossingPoint(prev, next, ep0, ep1)
      if (!ideal) continue

      const edgeLen = edge.length
      if (edgeLen < 1e-9) continue

      const edx = ep1.x - ep0.x
      const edy = ep1.y - ep0.y
      let t = ((ideal.x - ep0.x) * edx + (ideal.y - ep0.y) * edy) / (edgeLen * edgeLen)

      // Clamp: stay away from edge endpoints by at least spacing or 5%
      const tMin = Math.max(0.05, spacing / edgeLen)
      const tMax = Math.min(0.95, 1 - spacing / edgeLen)
      if (tMin >= tMax) continue // edge too short for this clearance
      t = Math.max(tMin, Math.min(tMax, t))

      // Respect crossing order
      const myIdx = edge.crossings.findIndex(
        (c) => c.connectionName === route.connectionName,
      )
      if (myIdx >= 0) {
        const minGap = spacing / edgeLen
        const prevCross = edge.crossings[myIdx - 1]
        const nextCross = edge.crossings[myIdx + 1]
        if (prevCross) t = Math.max(t, prevCross.t + minGap)
        if (nextCross) t = Math.min(t, nextCross.t - minGap)

        edge.crossings[myIdx]!.t = t
        edge.crossings[myIdx]!.point = {
          x: ep0.x + t * edx,
          y: ep0.y + t * edy,
        }
      }
    }
  }

  /**
   * Rebuild a route's path from the authoritative edge crossing positions.
   */
  private rebuildPathFromCrossings(cdt: RawCdt, route: TopoRouteState) {
    const edgesCrossed = (route as any)._edgesCrossed as number[] | undefined
    if (!edgesCrossed) return

    const newPath: Point[] = [route.start]
    for (const edgeIdx of edgesCrossed) {
      const edge = cdt.edges[edgeIdx]!
      const crossing = edge.crossings.find(
        (c) => c.connectionName === route.connectionName,
      )
      if (crossing) newPath.push({ ...crossing.point })
    }
    newPath.push(route.end)
    route.path = newPath
  }

  /**
   * Find the best point on edge c→d for rubber-banding between a and b.
   * If the line a→b crosses the edge, returns the intersection.
   * Otherwise, returns the point on the edge closest to line a→b.
   */
  private bestEdgeCrossingPoint(
    a: Point, b: Point, c: Point, d: Point,
  ): Point | null {
    const dx1 = b.x - a.x, dy1 = b.y - a.y
    const dx2 = d.x - c.x, dy2 = d.y - c.y
    const denom = dx1 * dy2 - dy1 * dx2

    if (Math.abs(denom) < 1e-12) {
      // Lines parallel — project midpoint of a,b onto edge c→d
      const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2
      const len2 = dx2 * dx2 + dy2 * dy2
      if (len2 < 1e-12) return null
      const t = Math.max(0, Math.min(1, ((mx - c.x) * dx2 + (my - c.y) * dy2) / len2))
      return { x: c.x + t * dx2, y: c.y + t * dy2 }
    }

    // Intersection of line a→b with line c→d
    const t2 = ((a.x - c.x) * dy1 - (a.y - c.y) * dx1) / denom
    const t2c = Math.max(0, Math.min(1, t2))
    return { x: c.x + t2c * dx2, y: c.y + t2c * dy2 }
  }

  static baseNetName(connectionName: string): string {
    const m = connectionName.match(/^(.+?)_mst\d+$/)
    return m ? m[1]! : connectionName
  }

  // ===== SOLVER STEP LOGIC =====

  _step() {
    switch (this.phase) {
      case "build-cdt":
        this.stepBuildCdt()
        break
      case "route":
        this.stepRoute()
        break
      case "rubberband":
        this.stepRubberBand()
        break
      case "commit":
        this.stepCommit()
        break
      case "done":
        this.solved = true
        break
    }
  }

  private stepBuildCdt() {
    this.cdts = []
    for (let z = 0; z < this.layerCount; z++) {
      const mergedRects = mergeOverlappingRects(
        this.baseObstaclePolygons[z]?.slice() ?? [],
      )
      const cdt = buildRawCdt(this.srj.bounds, mergedRects)
      this.cdts.push(cdt)
    }
    this.phase = "route"
    this.routeIndex = 0
  }

  private stepRoute() {
    // Route in batches
    const batchSize = 10
    const end = Math.min(this.routeIndex + batchSize, this.connections.length)

    for (let i = this.routeIndex; i < end; i++) {
      const conn = this.connections[i]!
      const route = this.routeConnection(conn)
      if (route) {
        this.routes.push(route)
      }
    }

    this.routeIndex = end
    if (this.routeIndex >= this.connections.length) {
      // STEP 1: Space all edge crossings evenly (gEDA space_edge)
      for (let z = 0; z < this.layerCount; z++) {
        const cdt = this.cdts[z]
        if (cdt) this.spaceAllEdges(cdt)
      }
      // STEP 2: Rebuild route paths from the spaced crossing positions
      for (const route of this.routes) {
        const cdt = this.cdts[route.routeLayerZ]
        if (cdt) this.rebuildPathFromCrossings(cdt, route)
      }
      this.phase = "rubberband"
      this.rubberBandIter = 0
    }
  }

  private routeConnection(
    conn: (typeof this.connections)[0],
  ): TopoRouteState | null {
    const preferredLayer = conn.startLayerZ
    const altLayer = preferredLayer === 0 ? 1 : 0

    const tryLayer = (layerZ: number, needStartVia: boolean, needEndVia: boolean) => {
      if (layerZ >= this.layerCount) return null
      const cdt = this.cdts[layerZ]
      if (!cdt) return null
      const result = this.routeThroughCdt(cdt, conn.start, conn.end, conn.name)
      if (!result) return null
      const routeIdx = this.routes.length
      this.recordEdgeCrossings(cdt, result.edgesCrossed, conn.name, routeIdx)
      const state: TopoRouteState = {
        connectionName: conn.name,
        path: result.path,
        routeLayerZ: layerZ,
        originalStart: conn.originalStart,
        originalEnd: conn.originalEnd,
        start: conn.start,
        end: conn.end,
        startLayerZ: conn.startLayerZ,
        endLayerZ: conn.endLayerZ,
        needsStartVia: needStartVia,
        needsEndVia: needEndVia,
      }
      ;(state as any)._edgesCrossed = result.edgesCrossed
      return state
    }

    // Try same-layer first
    if (conn.startLayerZ === conn.endLayerZ) {
      const r = tryLayer(conn.startLayerZ, false, false)
      if (r) return r
    }

    // Try each layer with vias
    for (const layerZ of [preferredLayer, altLayer]) {
      const r = tryLayer(layerZ, conn.startLayerZ !== layerZ, conn.endLayerZ !== layerZ)
      if (r) return r
    }

    return null
  }

  private stepRubberBand() {
    if (this.rubberBandIter >= 10) {
      this.finishRubberBand()
      return
    }

    let anyImproved = false

    // For each route, pull crossing points toward the straight-line ideal.
    // This modifies the edge crossing positions directly.
    for (const route of this.routes) {
      const cdt = this.cdts[route.routeLayerZ]
      if (!cdt) continue

      const oldLen = this.pathLength(route.path)
      this.rubberBandRoute(cdt, route)
      // Rebuild path from authoritative edge crossings
      this.rebuildPathFromCrossings(cdt, route)
      const newLen = this.pathLength(route.path)

      if (newLen < oldLen - 1e-6) anyImproved = true
    }

    this.rubberBandIter++
    if (!anyImproved) this.finishRubberBand()
  }

  private finishRubberBand() {
    // Simplify paths: remove crossing points where the straight line
    // from prev→next doesn't cross any constraint edge. This eliminates
    // zigzag from the CDT structure without changing the topological embedding.
    for (const route of this.routes) {
      const cdt = this.cdts[route.routeLayerZ]
      if (cdt) this.simplifyPath(cdt, route)
    }
    this.phase = "commit"
  }

  /**
   * Remove unnecessary crossing points from a route path.
   * A crossing is unnecessary if the straight line from its predecessor
   * to its successor doesn't cross any constraint edge (obstacle boundary).
   * This preserves the topological embedding while eliminating zigzag.
   *
   * Uses iterative greedy removal — keep removing points until no more
   * can be removed without crossing a constraint.
   */
  private simplifyPath(cdt: RawCdt, route: TopoRouteState) {
    // Collect constraint segments for intersection testing
    const constraintSegs: { x1: number; y1: number; x2: number; y2: number }[] = []
    for (const edge of cdt.edges) {
      if (!edge.isConstraint) continue
      const p0 = cdt.pts[edge.v0]!
      const p1 = cdt.pts[edge.v1]!
      constraintSegs.push({ x1: p0.x, y1: p0.y, x2: p1.x, y2: p1.y })
    }

    // Also treat other routes' path segments as soft obstacles for overlap prevention
    const otherSegs: { x1: number; y1: number; x2: number; y2: number; net: string }[] = []
    const baseNet = TopologicalPathSolver.baseNetName(route.connectionName)
    for (const other of this.routes) {
      if (other === route) continue
      if (other.routeLayerZ !== route.routeLayerZ) continue
      if (TopologicalPathSolver.baseNetName(other.connectionName) === baseNet) continue
      for (let i = 0; i < other.path.length - 1; i++) {
        const a = other.path[i]!, b = other.path[i + 1]!
        otherSegs.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y, net: other.connectionName })
      }
    }

    let changed = true
    while (changed) {
      changed = false
      const path = route.path
      for (let i = 1; i < path.length - 1; i++) {
        const prev = path[i - 1]!
        const next = path[i + 1]!

        // Check if we can skip this point
        let crossesConstraint = false
        for (const cs of constraintSegs) {
          if (this.segmentsIntersect(
            prev.x, prev.y, next.x, next.y,
            cs.x1, cs.y1, cs.x2, cs.y2,
          )) {
            crossesConstraint = true
            break
          }
        }

        if (!crossesConstraint) {
          // Safe to remove — splice it out
          path.splice(i, 1)
          // Also update _edgesCrossed
          const ec = (route as any)._edgesCrossed as number[] | undefined
          if (ec && i - 1 < ec.length) {
            ec.splice(i - 1, 1)
          }
          changed = true
          break // restart scan since indices shifted
        }
      }
    }
  }

  private segmentsIntersect(
    a1x: number, a1y: number, a2x: number, a2y: number,
    b1x: number, b1y: number, b2x: number, b2y: number,
  ): boolean {
    const d1x = a2x - a1x, d1y = a2y - a1y
    const d2x = b2x - b1x, d2y = b2y - b1y
    const denom = d1x * d2y - d1y * d2x
    if (Math.abs(denom) < 1e-12) return false
    const t = ((b1x - a1x) * d2y - (b1y - a1y) * d2x) / denom
    const u = ((b1x - a1x) * d1y - (b1y - a1y) * d1x) / denom
    return t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
  }

  /**
   * gEDA-style arc insertion: for each pair of consecutive path segments,
   * check if any obstacle vertex's clearance circle intersects the straight
   * line. If so, insert arc points that wrap around the obstacle vertex
   * at clearance distance.
   */
  private insertObstacleArcs(cdt: RawCdt, path: Point[]): Point[] {
    if (path.length < 2) return path

    const clearance = this.minTraceWidth / 2 + this.margin
    const result: Point[] = [path[0]!]

    for (let i = 0; i < path.length - 1; i++) {
      const a = path[i]!
      const b = path[i + 1]!

      // Find the obstacle vertex closest to segment a→b that violates clearance
      let worstVert = -1
      let worstDist = clearance

      for (let vi = 0; vi < cdt.pts.length; vi++) {
        if (!cdt.obstacleVertices.has(vi)) continue
        const v = cdt.pts[vi]!
        const d = this.pointToSegmentDist(v, a, b)
        if (d < worstDist) {
          // Make sure the vertex is actually between a and b (not past endpoints)
          const dx = b.x - a.x
          const dy = b.y - a.y
          const len2 = dx * dx + dy * dy
          if (len2 < 1e-12) continue
          const t = ((v.x - a.x) * dx + (v.y - a.y) * dy) / len2
          if (t > 0.05 && t < 0.95) {
            worstDist = d
            worstVert = vi
          }
        }
      }

      if (worstVert >= 0) {
        // Insert arc points around this obstacle vertex
        const v = cdt.pts[worstVert]!
        const arcPts = this.computeArcPoints(a, b, v, clearance)
        for (const ap of arcPts) {
          result.push(ap)
        }
      }

      result.push(b)
    }

    return result
  }

  /**
   * Compute arc points that route from a to b while staying clearance
   * distance from obstacle vertex v. Returns intermediate points
   * (not including a or b).
   */
  private computeArcPoints(
    a: Point, b: Point, v: Point, clearance: number,
  ): Point[] {
    // Direction from v to the line a→b
    const dx = b.x - a.x
    const dy = b.y - a.y
    const len = Math.hypot(dx, dy)
    if (len < 1e-9) return []

    // Perpendicular direction (which side of the line is v on?)
    const cross = (v.x - a.x) * dy - (v.y - a.y) * dx
    // If cross > 0, v is to the left of a→b; route goes to the right
    const sign = cross > 0 ? -1 : 1

    // Vector from v toward the line a→b, normalized
    const vToLineX = sign * (-dy / len)
    const vToLineY = sign * (dx / len)

    // Entry tangent point: project a onto circle around v
    const avx = a.x - v.x
    const avy = a.y - v.y
    const avLen = Math.hypot(avx, avy)

    const bvx = b.x - v.x
    const bvy = b.y - v.y
    const bvLen = Math.hypot(bvx, bvy)

    if (avLen < clearance * 1.1 || bvLen < clearance * 1.1) {
      // a or b is too close to v — just push through the perpendicular
      return [{
        x: v.x + vToLineX * clearance,
        y: v.y + vToLineY * clearance,
      }]
    }

    // Angle from v to a and v to b
    const angleA = Math.atan2(avy, avx)
    const angleB = Math.atan2(bvy, bvx)

    // Arc from entry to exit around v at clearance distance
    // Use 2-3 intermediate points
    let sweep = angleB - angleA
    // Normalize sweep to go around the side away from the line
    if (sign > 0) {
      while (sweep > 0) sweep -= Math.PI * 2
      while (sweep < -Math.PI * 2) sweep += Math.PI * 2
    } else {
      while (sweep < 0) sweep += Math.PI * 2
      while (sweep > Math.PI * 2) sweep -= Math.PI * 2
    }

    const steps = Math.max(2, Math.ceil(Math.abs(sweep) / (Math.PI / 4)))
    const pts: Point[] = []
    for (let s = 1; s < steps; s++) {
      const angle = angleA + (sweep * s) / steps
      pts.push({
        x: v.x + Math.cos(angle) * clearance,
        y: v.y + Math.sin(angle) * clearance,
      })
    }

    return pts
  }

  private stepCommit() {
    this.resolvedPaths = this.routes.map((route) => {
      const fullRoute: { x: number; y: number; z: number }[] = []
      const vias: { x: number; y: number }[] = []
      const routeStart = route.path[0]!
      const routeEnd = route.path[route.path.length - 1]!

      if (route.needsStartVia) {
        fullRoute.push({ x: route.originalStart.x, y: route.originalStart.y, z: route.startLayerZ })
        fullRoute.push({ x: routeStart.x, y: routeStart.y, z: route.startLayerZ })
        vias.push({ x: routeStart.x, y: routeStart.y })
      } else {
        fullRoute.push({ x: route.originalStart.x, y: route.originalStart.y, z: route.routeLayerZ })
      }

      for (const p of route.path) {
        fullRoute.push({ x: p.x, y: p.y, z: route.routeLayerZ })
      }

      if (route.needsEndVia) {
        vias.push({ x: routeEnd.x, y: routeEnd.y })
        fullRoute.push({ x: routeEnd.x, y: routeEnd.y, z: route.endLayerZ })
        fullRoute.push({ x: route.originalEnd.x, y: route.originalEnd.y, z: route.endLayerZ })
      } else {
        fullRoute.push({ x: route.originalEnd.x, y: route.originalEnd.y, z: route.routeLayerZ })
      }

      return { connectionName: route.connectionName, route: fullRoute, vias }
    })

    const routedSet = new Set(this.routes.map((r) => r.connectionName))
    this.validationResult = {
      totalConnections: this.connections.length,
      routedConnections: routedSet.size,
      unroutedConnections: this.connections
        .filter((c) => !routedSet.has(c.name))
        .map((c) => c.name),
      crossNetCrossings: [],
    }

    this.phase = "done"
    this.solved = true
  }

  private pathLength(path: Point[]): number {
    let len = 0
    for (let i = 1; i < path.length; i++) {
      len += distance(path[i - 1]!, path[i]!)
    }
    return len
  }

  getResolvedPaths(): ResolvedPath[] {
    return this.resolvedPaths
  }

  getEffectiveLayerCount(): number {
    return this.layerCount
  }

  getValidationResult() {
    return this.validationResult
  }

  visualize(): GraphicsObject {
    const lines: Line[] = []

    // Draw CDT edges (faint) for the first layer
    const cdt = this.cdts[0]
    if (cdt) {
      for (const edge of cdt.edges) {
        const p0 = cdt.pts[edge.v0]!
        const p1 = cdt.pts[edge.v1]!
        lines.push({
          points: [p0, p1],
          strokeColor: edge.isConstraint
            ? "rgba(255,0,0,0.15)"
            : "rgba(128,128,128,0.08)",
        })
      }
    }

    // Draw routes
    for (const route of this.routes) {
      if (route.path.length > 1) {
        const color = this.colorMap[route.connectionName] ?? "green"
        lines.push({
          points: route.path.map((p) => ({ x: p.x, y: p.y })),
          strokeColor: color,
          strokeWidth: this.minTraceWidth,
        })
      }
    }

    // Draw obstacles
    const rects = (this.srj.obstacles ?? []).map((o) => ({
      center: o.center,
      width: o.width,
      height: o.height,
      fill: o.layers?.includes("top")
        ? "rgba(255,0,0,0.15)"
        : "rgba(0,0,255,0.15)",
    }))

    const { minX, maxX, minY, maxY } = this.srj.bounds
    lines.push({
      points: [
        { x: minX, y: minY },
        { x: maxX, y: minY },
        { x: maxX, y: maxY },
        { x: minX, y: maxY },
        { x: minX, y: minY },
      ],
      strokeColor: "rgba(255,0,0,0.25)",
    })

    return { lines, rects }
  }
}

interface TopoRouteState {
  connectionName: string
  path: Point[]
  routeLayerZ: number
  originalStart: Point
  originalEnd: Point
  start: Point
  end: Point
  startLayerZ: number
  endLayerZ: number
  needsStartVia: boolean
  needsEndVia: boolean
}
