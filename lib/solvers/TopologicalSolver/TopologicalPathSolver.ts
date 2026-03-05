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
} from "./TopologicalCdt"
import { rubberbandSegment, arcsToPath } from "./TopoRubberBand"
import {
  type RouteVertex,
  createTempVertex,
  createFixedVertex,
  minSpacing,
} from "./TopoRouteVertex"

/**
 * Topological Rubberband Autorouter — direct port of gEDA toporouter.c
 *
 * Key principle: route vertices live ON CDT edges. The edge's routing
 * list (sorted by t-parameter) IS the topological state. Route paths
 * are linked lists of vertices via parent/child pointers.
 *
 * Algorithm phases:
 * 1. Build CDT from obstacles (once per layer)
 * 2. A* route each connection through CDT triangles, placing vertices on edges
 * 3. space_edge(): force-based relaxation to spread vertices on shared edges
 * 4. Commit paths to output format
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

  /** Per-layer raw CDTs */
  private cdts: (RawCdt | null)[] = []
  /** Per-layer base obstacle polygons */
  private baseObstaclePolygons: Point[][][] = []
  /** Per-layer edge routing lists: edgeIdx → sorted list of RouteVertex */
  private edgeRoutingLists: Map<number, RouteVertex[]>[] = []

  /** Connections to route */
  private connections: Array<{
    name: string
    originalStart: Point
    originalEnd: Point
    start: Point
    end: Point
    startLayerZ: number
    endLayerZ: number
  }> = []

  /** Committed route paths (ordered vertex lists) */
  private committedPaths: Array<{
    name: string
    vertices: RouteVertex[]
    layerZ: number
    originalStart: Point
    originalEnd: Point
    startLayerZ: number
    endLayerZ: number
  }> = []

  private resolvedPaths: ResolvedPath[] = []
  private phase: "build-cdt" | "route" | "space" | "rubberband" | "commit" | "done" = "build-cdt"
  private routeIndex = 0
  private layerNameToZ = new Map<string, number>()

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
    this.margin = params.margin ?? params.srj.defaultObstacleMargin ?? this.minTraceWidth
    this.viaDiameter = params.srj.minViaDiameter ?? 0.6
    const allLayerNames = this.getAllLayerNames()
    this.maxLayerCount = Math.max(params.srj.layerCount ?? 2, allLayerNames.length)
    this.layerCount = this.maxLayerCount

    const connMap = getConnectivityMapFromSimpleRouteJson(params.srj)
    this.colorMap = params.colorMap ?? getColorMap(params.srj, connMap)
    for (let z = 0; z < allLayerNames.length; z++) {
      this.layerNameToZ.set(allLayerNames[z]!, z)
    }

    this.baseObstaclePolygons = []
    for (let z = 0; z < this.maxLayerCount; z++) {
      const layerName = allLayerNames[z]!
      const polys: Point[][] = []
      for (const obs of params.srj.obstacles) {
        if (!obs.layers.includes(layerName)) continue
        polys.push(rectToPolygon(obs.center.x, obs.center.y, obs.width, obs.height, this.margin))
      }
      this.baseObstaclePolygons.push(polys)
    }

    // Build connections with nudged endpoints
    this.connections = params.srj.connections.map((conn) => {
      const pts = conn.pointsToConnect
      const originalStart = { x: pts[0]!.x, y: pts[0]!.y }
      const originalEnd = { x: pts[pts.length - 1]!.x, y: pts[pts.length - 1]!.y }

      const connNames = [conn.name]
      if (conn.rootConnectionName && conn.rootConnectionName !== conn.name)
        connNames.push(conn.rootConnectionName)
      if (conn.name.includes("__"))
        for (const part of conn.name.split("__"))
          if (!connNames.includes(part)) connNames.push(part)

      return {
        name: conn.name,
        originalStart,
        originalEnd,
        start: this.nudgeOutOfObstacle(originalStart, originalEnd, params.srj.obstacles, connNames),
        end: this.nudgeOutOfObstacle(originalEnd, originalStart, params.srj.obstacles, connNames),
        startLayerZ: this.connectionPointToLayerZ(pts[0]!),
        endLayerZ: this.connectionPointToLayerZ(pts[pts.length - 1]!),
      }
    })

    // Sort shortest first
    this.connections.sort((a, b) => distance(a.start, a.end) - distance(b.start, b.end))
  }

  // ===== UTILITIES =====

  private getAllLayerNames(): string[] {
    const s = new Set<string>()
    for (const obs of this.srj.obstacles) for (const l of obs.layers) s.add(l)
    for (const conn of this.srj.connections)
      for (const pt of conn.pointsToConnect) {
        if ("layer" in pt) s.add(pt.layer)
        if ("layers" in pt) for (const l of pt.layers) s.add(l)
      }
    const layers = Array.from(s)
    layers.sort((a, b) => {
      if (a === "top") return -1; if (b === "top") return 1
      if (a === "bottom") return 1; if (b === "bottom") return -1
      return a.localeCompare(b)
    })
    return layers.length > 0 ? layers : ["top", "bottom"]
  }

  private connectionPointToLayerZ(pt: ConnectionPoint): number {
    const layers = getConnectionPointLayers(pt)
    for (const name of layers) { const z = this.layerNameToZ.get(name); if (z !== undefined) return z }
    return 0
  }

  private nudgeOutOfObstacle(pt: Point, other: Point, obstacles: SimpleRouteJson["obstacles"], connNames: string[]): Point {
    const connected = obstacles.filter((obs) => connNames.some((n) => obs.connectedTo.includes(n)))
    if (connected.length === 0) return pt
    let containingObs: (typeof connected)[0] | null = null
    for (const obs of connected) {
      const hw = obs.width / 2 + this.margin + 0.05, hh = obs.height / 2 + this.margin + 0.05
      if (Math.abs(pt.x - obs.center.x) < hw && Math.abs(pt.y - obs.center.y) < hh) {
        if (!containingObs || obs.width * obs.height < containingObs.width * containingObs.height) containingObs = obs
      }
    }
    if (!containingObs) return pt
    const obs = containingObs
    const hw = obs.width / 2 + this.margin + 0.05, hh = obs.height / 2 + this.margin + 0.05
    const toX = other.x - pt.x, toY = other.y - pt.y
    const dx = pt.x - obs.center.x, dy = pt.y - obs.center.y
    const cands = [
      { x: obs.center.x + hw, y: pt.y, score: toX, dist: hw - dx },
      { x: obs.center.x - hw, y: pt.y, score: -toX, dist: hw + dx },
      { x: pt.x, y: obs.center.y + hh, score: toY, dist: hh - dy },
      { x: pt.x, y: obs.center.y - hh, score: -toY, dist: hh + dy },
    ]
    cands.sort((a, b) => { const aA = a.score > 0 ? 1 : 0, bA = b.score > 0 ? 1 : 0; return aA !== bA ? bA - aA : a.dist - b.dist })
    let result = { x: cands[0]!.x, y: cands[0]!.y }
    for (let iter = 0; iter < 5; iter++) {
      let pushed = false
      for (const o of connected) {
        const w = o.width / 2 + this.margin + 0.05, h = o.height / 2 + this.margin + 0.05
        const rx = result.x - o.center.x, ry = result.y - o.center.y
        if (Math.abs(rx) < w && Math.abs(ry) < h) {
          const dr = w - rx, dl = w + rx, dt = h - ry, db = h + ry
          const m = Math.min(dr, dl, dt, db)
          if (m === dr) result = { x: o.center.x + w, y: result.y }
          else if (m === dl) result = { x: o.center.x - w, y: result.y }
          else if (m === db) result = { x: result.x, y: o.center.y - h }
          else result = { x: result.x, y: o.center.y + h }
          pushed = true
        }
      }
      if (!pushed) break
    }
    return result
  }

  static baseNetName(connectionName: string): string {
    const m = connectionName.match(/^(.+?)_mst\d+$/)
    return m ? m[1]! : connectionName
  }

  // ===== CDT HELPERS =====

  private locateTriangle(cdt: RawCdt, pt: Point): number {
    for (let ti = 0; ti < cdt.triangles.length; ti++) {
      const tri = cdt.triangles[ti]!
      if (tri.obstacle) continue
      if (this.ptInTri(cdt.pts, tri.v, pt)) return ti
    }
    let bestD = Infinity, bestT = -1
    for (let ti = 0; ti < cdt.triangles.length; ti++) {
      const tri = cdt.triangles[ti]!; if (tri.obstacle) continue
      const cx = (cdt.pts[tri.v[0]]!.x + cdt.pts[tri.v[1]]!.x + cdt.pts[tri.v[2]]!.x) / 3
      const cy = (cdt.pts[tri.v[0]]!.y + cdt.pts[tri.v[1]]!.y + cdt.pts[tri.v[2]]!.y) / 3
      const d = (pt.x - cx) ** 2 + (pt.y - cy) ** 2
      if (d < bestD) { bestD = d; bestT = ti }
    }
    return bestT
  }

  private ptInTri(pts: Point[], v: [number, number, number], p: Point): boolean {
    const a = pts[v[0]]!, b = pts[v[1]]!, c = pts[v[2]]!
    const d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y)
    const d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y)
    const d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y)
    return !(d1 < 0 && (d2 > 0 || d3 > 0)) && !(d1 > 0 && (d2 < 0 || d3 < 0))
  }

  private edgeKey(a: number, b: number): string {
    return a < b ? `${a},${b}` : `${b},${a}`
  }

  /** Winding: positive = left, negative = right, 0 = collinear */
  private wind(ax: number, ay: number, bx: number, by: number, cx: number, cy: number): number {
    const cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if (cross > 1e-9) return 1
    if (cross < -1e-9) return -1
    return 0
  }

  /** Get edge routing list for a given layer/edge */
  private getEdgeRouting(layerZ: number, edgeIdx: number): RouteVertex[] {
    if (!this.edgeRoutingLists[layerZ]) return []
    return this.edgeRoutingLists[layerZ]!.get(edgeIdx) ?? []
  }

  /** Insert a vertex into an edge's routing list (sorted by t) */
  private insertIntoEdgeRouting(layerZ: number, edgeIdx: number, rv: RouteVertex) {
    if (!this.edgeRoutingLists[layerZ]) this.edgeRoutingLists[layerZ] = new Map()
    const list = this.edgeRoutingLists[layerZ]!.get(edgeIdx) ?? []
    // Insert sorted by t
    let idx = 0
    while (idx < list.length && list[idx]!.t < rv.t) idx++
    list.splice(idx, 0, rv)
    this.edgeRoutingLists[layerZ]!.set(edgeIdx, list)
  }

  /** Edge capacity = geometric length */
  private edgeCapacity(cdt: RawCdt, edgeIdx: number): number {
    return cdt.edges[edgeIdx]!.length
  }

  /** Edge flow = total spacing consumed by existing route vertices */
  private edgeFlow(layerZ: number, cdt: RawCdt, edgeIdx: number, destThickness: number): number {
    const routing = this.getEdgeRouting(layerZ, edgeIdx)
    if (routing.length === 0) return 0
    const edge = cdt.edges[edgeIdx]!
    let flow = 0
    const ep0 = cdt.pts[edge.v0]!, ep1 = cdt.pts[edge.v1]!

    // Flow from edge v0 to first routing vertex
    flow += minSpacing({ thickness: 0 }, routing[0]!, this.margin)
    // Flow between consecutive routing vertices
    for (let i = 0; i < routing.length - 1; i++) {
      flow += minSpacing(routing[i]!, routing[i + 1]!, this.margin)
    }
    // Flow from last routing vertex to edge v1
    flow += minSpacing(routing[routing.length - 1]!, { thickness: 0 }, this.margin)
    return flow
  }

  // ===== CANDIDATE VERTEX GENERATION (gEDA candidate_vertices) =====

  /**
   * Generate candidate routing vertices in the gap between v1pos and v2pos
   * on the given edge. Returns up to 3 candidates.
   */
  private candidateVertices(
    cdt: RawCdt,
    layerZ: number,
    edgeIdx: number,
    v1t: number,
    v2t: number,
    destThickness: number,
  ): RouteVertex[] {
    const edge = cdt.edges[edgeIdx]!
    const ep0 = cdt.pts[edge.v0]!, ep1 = cdt.pts[edge.v1]!
    const edgeLen = edge.length
    if (edgeLen < 1e-9) return []

    // Check capacity
    const flow = this.edgeFlow(layerZ, cdt, edgeIdx, destThickness)
    if (flow >= edgeLen) return []

    const gapLen = Math.abs(v2t - v1t) * edgeLen
    const ms = this.minTraceWidth + this.margin
    const v1ms = ms // spacing from v1 side
    const v2ms = ms // spacing from v2 side

    const tLow = Math.min(v1t, v2t)
    const tHigh = Math.max(v1t, v2t)

    if (gapLen < ms) {
      // Gap too small to fit any vertex with clearance
      return []
    }

    if (v1ms + v2ms + ms >= gapLen) {
      // Tight: single midpoint (gEDA behavior)
      const tMid = (tLow + tHigh) / 2
      return [createTempVertex(edgeIdx, tMid, ep0, ep1, destThickness)]
    }

    const results: RouteVertex[] = []
    const t1 = tLow + v1ms / edgeLen
    const t2 = tHigh - v2ms / edgeLen
    results.push(createTempVertex(edgeIdx, t1, ep0, ep1, destThickness))
    results.push(createTempVertex(edgeIdx, t2, ep0, ep1, destThickness))

    // Center candidate if space remains
    const centerT = (t1 + t2) / 2
    if (Math.abs(t2 - t1) * edgeLen > ms) {
      results.push(createTempVertex(edgeIdx, centerT, ep0, ep1, destThickness))
    }

    return results
  }

  // ===== A* ROUTING (gEDA route()) =====

  /**
   * Find the nearest CDT vertex to a point. Used to map terminal
   * positions to CDT vertices for the direct-connection check.
   */
  private nearestCdtVertex(cdt: RawCdt, pt: Point): number {
    let bestD = Infinity, bestV = -1
    for (let vi = 0; vi < cdt.pts.length; vi++) {
      const d = (cdt.pts[vi]!.x - pt.x) ** 2 + (cdt.pts[vi]!.y - pt.y) ** 2
      if (d < bestD) { bestD = d; bestV = vi }
    }
    return bestV
  }

  /**
   * Route one connection through the CDT using A*.
   * Direct port of gEDA route():
   * - Uses vertex identity (not coordinate strings) for closed set
   * - Direct connection check (gts_vertices_are_connected)
   * - Proper candidate generation matching gEDA's triangle traversal
   */
  private routeConnection(
    cdt: RawCdt,
    layerZ: number,
    conn: (typeof this.connections)[0],
  ): RouteVertex[] | null {
    const startTri = this.locateTriangle(cdt, conn.start)
    const endTri = this.locateTriangle(cdt, conn.end)
    if (startTri < 0 || endTri < 0) return null

    const srcVertex = createFixedVertex(conn.start.x, conn.start.y, this.minTraceWidth)
    const destVertex = createFixedVertex(conn.end.x, conn.end.y, this.minTraceWidth)
    srcVertex.gcost = 0
    srcVertex.hcost = distance(conn.start, conn.end)

    // Map destination to nearest CDT vertex for direct-connection checks
    const destCdtVi = this.nearestCdtVertex(cdt, conn.end)

    const open: RouteVertex[] = [srcVertex]
    // Position-based closed set: "edgeIdx:t" for edge vertices, "x,y" for terminals
    const closed = new Set<string>()
    // Position-based vertex cache: reuse vertices at same position
    const vertexCache = new Map<string, RouteVertex>()
    let iterCount = 0

    const posKey = (rv: RouteVertex) =>
      rv.edgeIdx >= 0 ? `e${rv.edgeIdx}:${rv.t.toFixed(6)}` : `${rv.x.toFixed(4)},${rv.y.toFixed(4)}`

    const routeStartTime = Date.now()
    while (open.length > 0 && iterCount < 5000) {
      iterCount++
      if (Date.now() - routeStartTime > 2000) {
        console.warn(`TopologicalPathSolver: A* timeout for ${conn.name} after ${iterCount} iters, open=${open.length} closed=${closed.size} cache=${vertexCache.size}`)
        return null
      }
      // Pop lowest f-cost
      let bestIdx = 0
      for (let i = 1; i < open.length; i++) {
        if ((open[i]!.gcost + open[i]!.hcost) < (open[bestIdx]!.gcost + open[bestIdx]!.hcost)) bestIdx = i
      }
      const cur = open[bestIdx]!
      open.splice(bestIdx, 1)

      const curKey = posKey(cur)
      if (closed.has(curKey)) continue
      closed.add(curKey)

      // Check if we reached destination (by position, not object identity)
      if (distance(cur, destVertex) < 0.1) {
        // cur IS at the destination — build path
        const path: RouteVertex[] = []
        let v: RouteVertex | null = cur
        while (v) { path.push(v); v = v.parent }
        path.reverse()
        // Ensure last point is exactly at dest
        if (path.length > 0) {
          const last = path[path.length - 1]!
          if (distance(last, destVertex) > 0.01) {
            path.push(destVertex)
            destVertex.parent = last
          }
        }
        return path
      }

      // Generate candidates
      const candidates = this.computeCandidatePoints(cdt, layerZ, cur, destVertex, destCdtVi)

      for (const cand of candidates) {
        const candKey = posKey(cand)
        if (closed.has(candKey)) continue

        // Check if segment cur→cand would overlap any existing route
        if (this.segmentOverlapsExistingRoute(layerZ, cur, cand, conn.name)) continue

        const g = cur.gcost + distance(cur, cand)
        const h = distance(cand, destVertex)

        // Reuse existing vertex at same position
        const existing = vertexCache.get(candKey)
        if (existing) {
          if (g < existing.gcost) {
            existing.gcost = g
            existing.hcost = h
            existing.parent = cur
          }
          continue
        }

        cand.gcost = g
        cand.hcost = h
        cand.parent = cur
        vertexCache.set(candKey, cand)
        open.push(cand)
      }
    }

    if (iterCount >= 20000) {
      console.warn(`TopologicalPathSolver: A* exhausted 20000 iterations for ${conn.name}, open=${open.length} closed=${closed.size}`)
    }
    return null
  }

  /**
   * Check if adding a path segment from a→b would overlap any existing
   * committed route on the same layer. Returns true if overlap detected.
   */
  private segmentOverlapsExistingRoute(
    layerZ: number,
    a: Point,
    b: Point,
    connectionName: string,
  ): boolean {
    const minDist = this.minTraceWidth
    const baseNet = TopologicalPathSolver.baseNetName(connectionName)

    for (const cp of this.committedPaths) {
      if (cp.layerZ !== layerZ) continue
      if (TopologicalPathSolver.baseNetName(cp.name) === baseNet) continue

      const verts = cp.vertices
      for (let i = 0; i < verts.length - 1; i++) {
        const c = verts[i]!, d = verts[i + 1]!
        const dist = this.segSegMinDist(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y)
        if (dist < minDist) return true
      }
    }
    return false
  }

  private segSegMinDist(
    a1x: number, a1y: number, a2x: number, a2y: number,
    b1x: number, b1y: number, b2x: number, b2y: number,
  ): number {
    const ptSeg = (px: number, py: number, sx: number, sy: number, ex: number, ey: number) => {
      const dx = ex - sx, dy = ey - sy, len2 = dx * dx + dy * dy
      if (len2 < 1e-12) return Math.hypot(px - sx, py - sy)
      const t = Math.max(0, Math.min(1, ((px - sx) * dx + (py - sy) * dy) / len2))
      return Math.hypot(px - (sx + t * dx), py - (sy + t * dy))
    }
    return Math.min(
      ptSeg(a1x, a1y, b1x, b1y, b2x, b2y),
      ptSeg(a2x, a2y, b1x, b1y, b2x, b2y),
      ptSeg(b1x, b1y, a1x, a1y, a2x, a2y),
      ptSeg(b2x, b2y, a1x, a1y, a2x, a2y),
    )
  }

  /**
   * Generate candidate points — port of gEDA compute_candidate_points().
   *
   * Key difference from previous implementation:
   * - From a CDT vertex: check direct connection first, then only generate
   *   candidates on the OPPOSITE edge of each adjacent triangle
   * - From a temp vertex on edge: winding check, then candidates on the
   *   two other edges of the opposite-side triangle
   */
  private computeCandidatePoints(
    cdt: RawCdt,
    layerZ: number,
    cur: RouteVertex,
    dest: RouteVertex,
    destCdtVi: number,
  ): RouteVertex[] {
    const candidates: RouteVertex[] = []

    if (cur.edgeIdx < 0) {
      // Fixed vertex or CDT vertex — find nearest CDT vertex
      const curCdtVi = this.nearestCdtVertex(cdt, cur)

      // gEDA direct connection check:
      // If curpoint is directly connected to dest via a non-constraint,
      // non-routed CDT edge, go straight there
      if (curCdtVi >= 0 && destCdtVi >= 0 && cdt.vertexNeighbors[curCdtVi]?.has(destCdtVi)) {
        const ek = this.edgeKey(curCdtVi, destCdtVi)
        const ei = cdt.edgeMap.get(ek)
        if (ei !== undefined) {
          const edge = cdt.edges[ei]!
          if (!edge.isConstraint && this.getEdgeRouting(layerZ, ei).length === 0) {
            // Direct connection! Return dest as only candidate
            return [dest]
          }
        }
      }

      // Explore all adjacent triangles (gEDA: gts_vertex_triangles)
      // For each triangle, generate candidates on the OPPOSITE edge only
      // (gEDA triangle_candidate_points_from_vertex)
      const tris = cdt.vertexTriangles[curCdtVi] ?? this.findTrianglesContainingPoint(cdt, cur)
      for (const ti of tris) {
        const tri = cdt.triangles[ti]!
        if (tri.obstacle) continue

        // Find the opposite edge (the one edge that doesn't contain curCdtVi)
        for (let slot = 0; slot < 3; slot++) {
          const va = tri.v[slot]!, vb = tri.v[(slot + 1) % 3]!
          if (va === curCdtVi || vb === curCdtVi) continue

          // This is the opposite edge
          const ek = this.edgeKey(va, vb)
          const ei = cdt.edgeMap.get(ek)
          if (ei === undefined) continue
          const edge = cdt.edges[ei]!

          // gEDA: constraint edges of different nets are impassable
          if (edge.isConstraint) continue

          // Check if dest vertex is on this edge
          if (va === destCdtVi || vb === destCdtVi) {
            candidates.push(dest)
            continue
          }

          const cands = this.candidateVerticesOnEdge(cdt, layerZ, ei, dest.thickness)
          candidates.push(...cands)
          break // Only one opposite edge per triangle
        }
      }
    } else {
      // Temp vertex on edge — winding check (gEDA prevwind)
      const edge = cdt.edges[cur.edgeIdx]!
      const ep0 = cdt.pts[edge.v0]!, ep1 = cdt.pts[edge.v1]!

      const parentWind = cur.parent
        ? this.wind(ep0.x, ep0.y, ep1.x, ep1.y, cur.parent.x, cur.parent.y)
        : 0

      // Only explore the triangle on the OPPOSITE side from parent
      for (const triIdx of [edge.t0, edge.t1]) {
        if (triIdx < 0) continue
        const tri = cdt.triangles[triIdx]!
        if (tri.obstacle) continue

        const oppIdx = tri.v.find((vi) => vi !== edge.v0 && vi !== edge.v1)
        if (oppIdx === undefined) continue
        const oppV = cdt.pts[oppIdx]!

        const oppWind = this.wind(ep0.x, ep0.y, ep1.x, ep1.y, oppV.x, oppV.y)
        if (parentWind !== 0 && oppWind === parentWind) continue

        // gEDA: check if dest is the opposite vertex
        if (oppIdx === destCdtVi) {
          candidates.push(dest)
          break // gEDA breaks after finding the opposite-side triangle
        }

        // gEDA triangle_candidate_points_from_edge:
        // Generate candidates on the OPPOSITE edge of this triangle
        // (the edge that doesn't share a vertex with cur.edgeIdx's endpoints
        //  AND doesn't share a vertex with cur.edgeIdx)
        // In practice: the opposite edge from the entry edge in this triangle
        // is the edge connecting the two vertices that aren't on cur.edgeIdx
        // But since oppIdx IS the vertex not on cur.edgeIdx, the opposite edge
        // is the one connecting the two cur.edgeIdx endpoints: edge.v0→edge.v1
        // which IS cur.edgeIdx. So actually we want the two edges connecting
        // oppIdx to edge.v0 and edge.v1 respectively.
        //
        // gEDA generates candidates on op_e (edge opposite to vertex v in triangle)
        // When expanding from edge E, the "vertex" is oppIdx, so op_e is E itself.
        // But we already came from E! So gEDA actually generates on e1 and e2
        // (the edges from oppV to E's endpoints), finding gaps via parent/child.
        //
        // For now: generate on both side edges (e1, e2) but with proper gap finding
        for (let slot = 0; slot < 3; slot++) {
          const va = tri.v[slot]!, vb = tri.v[(slot + 1) % 3]!
          const ek = this.edgeKey(va, vb)
          const ei = cdt.edgeMap.get(ek)
          if (ei === undefined || ei === cur.edgeIdx) continue
          const e = cdt.edges[ei]!
          if (e.isConstraint) continue

          const cands = this.candidateVerticesOnEdge(cdt, layerZ, ei, dest.thickness)
          candidates.push(...cands)
        }

        break // gEDA breaks after finding the opposite-side triangle
      }
    }

    return candidates
  }

  /** Fallback: find triangles containing a point by brute force */
  private findTrianglesContainingPoint(cdt: RawCdt, pt: Point): number[] {
    const tris: number[] = []
    for (let ti = 0; ti < cdt.triangles.length; ti++) {
      const tri = cdt.triangles[ti]!
      if (tri.obstacle) continue
      if (this.ptInTri(cdt.pts, tri.v, pt)) tris.push(ti)
    }
    if (tris.length === 0) {
      const nearest = this.locateTriangle(cdt, pt)
      if (nearest >= 0) tris.push(nearest)
    }
    return tris
  }

  /**
   * Generate candidate vertices along an edge, finding gaps between
   * existing route vertices (gEDA candidate_vertices).
   */
  private candidateVerticesOnEdge(
    cdt: RawCdt,
    layerZ: number,
    edgeIdx: number,
    destThickness: number,
  ): RouteVertex[] {
    const routing = this.getEdgeRouting(layerZ, edgeIdx)
    const candidates: RouteVertex[] = []

    if (routing.length === 0) {
      // No existing routing — candidates across full edge
      return this.candidateVertices(cdt, layerZ, edgeIdx, 0, 1, destThickness)
    }

    // Find gaps between existing route vertices
    // Gap from edge start (t=0) to first route vertex
    candidates.push(...this.candidateVertices(cdt, layerZ, edgeIdx, 0, routing[0]!.t, destThickness))

    // Gaps between consecutive route vertices
    for (let i = 0; i < routing.length - 1; i++) {
      candidates.push(...this.candidateVertices(cdt, layerZ, edgeIdx, routing[i]!.t, routing[i + 1]!.t, destThickness))
    }

    // Gap from last route vertex to edge end (t=1)
    candidates.push(...this.candidateVertices(cdt, layerZ, edgeIdx, routing[routing.length - 1]!.t, 1, destThickness))

    return candidates
  }

  // ===== SPACE EDGES (gEDA space_edge()) =====

  /**
   * Force-based relaxation to spread route vertices evenly on shared edges.
   * 100 iterations with 0.1 damping, like gEDA.
   */
  private spaceEdge(cdt: RawCdt, layerZ: number, edgeIdx: number) {
    const edge = cdt.edges[edgeIdx]!
    if (edge.isConstraint) return

    const routing = this.getEdgeRouting(layerZ, edgeIdx)
    if (routing.length === 0) return

    const ep0 = cdt.pts[edge.v0]!, ep1 = cdt.pts[edge.v1]!
    const edgeLen = edge.length
    if (edgeLen < 1e-9) return

    const forces = new Float64Array(routing.length)

    for (let iter = 0; iter < 100; iter++) {
      let equilibrium = true

      // Compute forces
      for (let k = 0; k < routing.length; k++) {
        const v = routing[k]!
        let force = 0

        // Force from previous (or edge start)
        const prevT = k > 0 ? routing[k - 1]!.t : 0
        const prevThickness = k > 0 ? routing[k - 1]!.thickness : 0
        const ms1 = minSpacing(v, { thickness: prevThickness }, this.margin)
        const d1 = (v.t - prevT) * edgeLen
        if (d1 < ms1) force += (ms1 - d1) // push toward v1 direction

        // Force from next (or edge end)
        const nextT = k < routing.length - 1 ? routing[k + 1]!.t : 1
        const nextThickness = k < routing.length - 1 ? routing[k + 1]!.thickness : 0
        const ms2 = minSpacing(v, { thickness: nextThickness }, this.margin)
        const d2 = (nextT - v.t) * edgeLen
        if (d2 < ms2) force -= (ms2 - d2) // push toward v0 direction

        forces[k] = force
      }

      // Apply forces with damping
      for (let k = 0; k < routing.length; k++) {
        if (Math.abs(forces[k]!) > 1e-6) equilibrium = false
        const dt = (forces[k]! * 0.1) / edgeLen // convert distance to t-delta
        routing[k]!.t += dt
        routing[k]!.t = Math.max(0.01, Math.min(0.99, routing[k]!.t))
        // Update position
        routing[k]!.x = ep0.x + routing[k]!.t * (ep1.x - ep0.x)
        routing[k]!.y = ep0.y + routing[k]!.t * (ep1.y - ep0.y)
      }

      if (equilibrium) break
    }
  }

  // ===== SOLVER PHASES =====

  _step() {
    switch (this.phase) {
      case "build-cdt": this.stepBuildCdt(); break
      case "route": this.stepRoute(); break
      case "space": this.stepSpace(); break
      case "rubberband": this.stepRubberBand(); break
      case "commit": this.stepCommit(); break
      case "done": this.solved = true; break
    }
  }

  private stepBuildCdt() {
    this.cdts = []
    this.edgeRoutingLists = []
    for (let z = 0; z < this.layerCount; z++) {
      const merged = mergeOverlappingRects(this.baseObstaclePolygons[z]?.slice() ?? [])
      this.cdts.push(buildRawCdt(this.srj.bounds, merged))
      this.edgeRoutingLists.push(new Map())
    }
    this.phase = "route"
    this.routeIndex = 0
  }

  private stepRoute() {
    const batchSize = 5
    const end = Math.min(this.routeIndex + batchSize, this.connections.length)

    for (let i = this.routeIndex; i < end; i++) {
      const conn = this.connections[i]!
      const preferredLayer = conn.startLayerZ
      const altLayer = preferredLayer === 0 ? 1 : 0

      let routed = false
      const tryLayer = (lz: number) => {
        if (lz >= this.layerCount || routed) return
        const cdt = this.cdts[lz]
        if (!cdt) return
        const path = this.routeConnection(cdt, lz, conn)
        if (!path) return

        // Commit: insert route vertices into edge routing lists (gEDA apply_route)
        for (const rv of path) {
          rv.routeName = conn.name
          rv.isTemp = false
          if (rv.edgeIdx >= 0) {
            this.insertIntoEdgeRouting(lz, rv.edgeIdx, rv)
          }
        }

        // Link parent/child
        for (let j = 0; j < path.length - 1; j++) {
          path[j]!.child = path[j + 1]!
          path[j + 1]!.parent = path[j]!
        }

        this.committedPaths.push({
          name: conn.name,
          vertices: path,
          layerZ: lz,
          originalStart: conn.originalStart,
          originalEnd: conn.originalEnd,
          startLayerZ: conn.startLayerZ,
          endLayerZ: conn.endLayerZ,
        })
        routed = true
      }

      if (conn.startLayerZ === conn.endLayerZ) tryLayer(conn.startLayerZ)
      if (!routed && preferredLayer !== conn.startLayerZ) tryLayer(preferredLayer)
      if (!routed && altLayer !== conn.startLayerZ && altLayer !== preferredLayer) tryLayer(altLayer)
    }

    this.routeIndex = end
    if (this.routeIndex >= this.connections.length) {
      this.phase = "space"
    }
  }

  private stepSpace() {
    // gEDA: space_edge on all edges after all routes committed
    for (let z = 0; z < this.layerCount; z++) {
      const cdt = this.cdts[z]
      if (!cdt) continue
      for (let ei = 0; ei < cdt.edges.length; ei++) {
        this.spaceEdge(cdt, z, ei)
      }
    }
    this.phase = "rubberband"
  }

  private stepRubberBand() {
    // gEDA: oproute_rubberband() for each route after space_edge
    for (const cp of this.committedPaths) {
      const cdt = this.cdts[cp.layerZ]
      if (!cdt) continue
      if (cp.vertices.length < 3) continue // need at least start + edge + end

      const start = cp.vertices[0]!
      const end = cp.vertices[cp.vertices.length - 1]!

      // Create arcs around obstacle vertices that violate clearance
      const arcs = rubberbandSegment(
        cdt,
        cp.vertices,
        1,
        cp.vertices.length - 1,
        { kind: "point", x: start.x, y: start.y },
        { kind: "point", x: end.x, y: end.y },
        this.margin,
        this.minTraceWidth,
      )

      if (arcs.length > 0) {
        // Replace path with arc-based smooth path
        const smoothPath = arcsToPath(
          { x: start.x, y: start.y },
          { x: end.x, y: end.y },
          arcs,
        )

        cp.vertices = smoothPath.map((p, i) => {
          if (i === 0) return cp.vertices[0]!
          if (i === smoothPath.length - 1) return cp.vertices[cp.vertices.length - 1]!
          return {
            x: p.x, y: p.y,
            edgeIdx: -1, t: -1,
            isTemp: false, parent: null, child: null,
            gcost: 0, hcost: 0,
            routeName: cp.name, thickness: this.minTraceWidth,
          } as RouteVertex
        })
      }
    }

    this.phase = "commit"
  }

  private stepCommit() {
    this.resolvedPaths = this.committedPaths.map((cp) => {
      const fullRoute: { x: number; y: number; z: number }[] = []
      const vias: { x: number; y: number }[] = []

      const needStartVia = cp.startLayerZ !== cp.layerZ
      const needEndVia = cp.endLayerZ !== cp.layerZ

      // gEDA: the path includes terminal vertices at both ends.
      // Bridges from originalStart/End are handled by the output solver
      // which knows the connection endpoints. We just emit the route
      // with vias where layer transitions occur.

      if (needStartVia) {
        // Start on native layer, via to route layer
        fullRoute.push({ x: cp.originalStart.x, y: cp.originalStart.y, z: cp.startLayerZ })
        vias.push({ x: cp.vertices[0]!.x, y: cp.vertices[0]!.y })
      }

      // Main route on route layer
      for (const rv of cp.vertices) {
        fullRoute.push({ x: rv.x, y: rv.y, z: cp.layerZ })
      }

      if (needEndVia) {
        // Via to end native layer
        const last = cp.vertices[cp.vertices.length - 1]!
        vias.push({ x: last.x, y: last.y })
        fullRoute.push({ x: cp.originalEnd.x, y: cp.originalEnd.y, z: cp.endLayerZ })
      }

      return { connectionName: cp.name, route: fullRoute, vias }
    })

    const routedSet = new Set(this.committedPaths.map((p) => p.name))
    this.validationResult = {
      totalConnections: this.connections.length,
      routedConnections: routedSet.size,
      unroutedConnections: this.connections.filter((c) => !routedSet.has(c.name)).map((c) => c.name),
      crossNetCrossings: [],
    }

    this.phase = "done"
    this.solved = true
  }

  // ===== PUBLIC API =====

  getResolvedPaths(): ResolvedPath[] { return this.resolvedPaths }
  getEffectiveLayerCount(): number { return this.layerCount }
  getValidationResult() { return this.validationResult }

  visualize(): GraphicsObject {
    const lines: Line[] = []

    // CDT edges
    const cdt = this.cdts[0]
    if (cdt) {
      for (const edge of cdt.edges) {
        lines.push({
          points: [cdt.pts[edge.v0]!, cdt.pts[edge.v1]!],
          strokeColor: edge.isConstraint ? "rgba(255,0,0,0.15)" : "rgba(128,128,128,0.06)",
        })
      }
    }

    // Routes
    for (const cp of this.committedPaths) {
      if (cp.vertices.length > 1) {
        lines.push({
          points: cp.vertices.map((v) => ({ x: v.x, y: v.y })),
          strokeColor: this.colorMap[cp.name] ?? "green",
          strokeWidth: this.minTraceWidth,
        })
      }
    }

    const rects = (this.srj.obstacles ?? []).map((o) => ({
      center: o.center, width: o.width, height: o.height,
      fill: o.layers?.includes("top") ? "rgba(255,0,0,0.15)" : "rgba(0,0,255,0.15)",
    }))

    const { minX, maxX, minY, maxY } = this.srj.bounds
    lines.push({
      points: [{ x: minX, y: minY }, { x: maxX, y: minY }, { x: maxX, y: maxY }, { x: minX, y: maxY }, { x: minX, y: minY }],
      strokeColor: "rgba(255,0,0,0.25)",
    })

    return { lines, rects }
  }
}
