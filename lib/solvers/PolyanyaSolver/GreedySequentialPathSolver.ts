import type { GraphicsObject, Line } from "graphics-debug"
import {
  type Mesh,
  type Point,
  type WeightedRegion,
  VisibilityGraph,
  SearchInstance,
  cdtTriangulate,
  rectToPolygon,
  buildMeshFromRegions,
  mergeMesh,
} from "polyanya"
import { BaseSolver } from "../BaseSolver"
import type { SimpleRouteJson, ConnectionPoint } from "../../types"
import { getConnectionPointLayers } from "../../utils/connection-point-utils"
import type { ResolvedPath } from "./types"
import { mergeOverlappingRects } from "./mergeOverlappingRects"
import {
  TracePhysicsRelaxer,
  type RelaxerObstacle,
} from "./TracePhysicsRelaxer"

const TRACE_WEIGHT = 10
const TRACE_PENALTY = 25

// Phase ordering: shortest-first by default, longest-first available as option
type Phase = "shortest" | "longest" | "done"

/**
 * Greedy sequential path solver: routes one trace at a time using the
 * visibility graph algorithm. Each committed trace is written as either
 * a hard obstacle or a weighted penalty region (configurable via
 * `useObstacles`) so future traces avoid it.
 *
 * Supports multiple layers. Layer 0 ("top") has all top-layer SRJ obstacles.
 * Layer 1+ only have obstacles present on that layer (vias, through-hole
 * pins, and traces routed on that layer). When a trace can't route on
 * layer 0, it falls back to layer 1. If it can't route on any layer,
 * it triggers the shortest→longest phase fallback.
 *
 * Tries shortest-first ordering. If it gets stuck, resets and tries
 * longest-first, then keeps whichever routed more traces. When stuck
 * on all layers, adds a new layer (up to maxLayerCount) before giving up.
 */
export class GreedySequentialPathSolver extends BaseSolver {
  private srj: SimpleRouteJson
  private colorMap: Record<string, string>
  private minTraceWidth: number
  private margin: number
  private viaDiameter: number
  private layerCount: number

  /** When true, committed traces become hard obstacles and the mesh is
   *  rebuilt after each trace. When false, traces become weighted regions
   *  and the mesh is built once. */
  private useObstacles: boolean

  /** When true, use Polyanya SearchInstance for pathfinding (no weighted
   *  regions). When false, use VisibilityGraph (supports weighted regions). */
  private usePolyanya: boolean

  /** All connections (immutable reference for resets) */
  private allConnections: Array<{
    name: string
    start: Point
    end: Point
    originalStart: Point
    originalEnd: Point
    startCandidates: Point[]
    endCandidates: Point[]
    startLayerZ: number
    endLayerZ: number
    congestionScore: number
    /** All names that might match an obstacle's connectedTo list */
    connNames: string[]
  }>

  /** Connections still waiting to be routed */
  private remaining: Array<{
    name: string
    start: Point
    end: Point
    originalStart: Point
    originalEnd: Point
    startCandidates: Point[]
    endCandidates: Point[]
    startLayerZ: number
    endLayerZ: number
    congestionScore: number
    connNames: string[]
  }>

  /** Per-layer base obstacle polygons from original SRJ only (for resets) */
  private baseObstaclePolygons: Point[][][]

  /** Per-layer rect obstacle polygons (base rects + via rects, merged before CDT) */
  private rectObstacles: Point[][][] = []

  /** Per-layer: connectedTo names for each base obstacle polygon (parallel to baseObstaclePolygons) */
  private baseObstacleConnectedTo: string[][] = []

  /** Per-layer: how many entries at the start of rectObstacles[z] are base obstacles */
  private baseObstacleCount: number[] = []

  /** Per-layer cache: connNames key → Mesh built with those obstacles removed.
   *  Invalidated whenever buildMesh() is called. */
  private connectionMeshCache: Map<string, Mesh | null>[] = []

  /** Per-layer trace obstacle polygons (thick polyline offsets, passed directly to CDT without merging) */
  private tracePolygonObstacles: Point[][][] = []

  /** Cached AABBs for trace obstacle polygons (parallel to tracePolygonObstacles) */
  private tracePolyAABBs: {
    minX: number
    minY: number
    maxX: number
    maxY: number
  }[][] = []

  /** Base net name for each trace obstacle polygon (parallel to tracePolygonObstacles) */
  private tracePolyNetNames: string[][] = []

  /** Per-layer weighted regions from committed traces (only used when !useObstacles) */
  private traceWeightedRegions: WeightedRegion[][] = []

  /** Per-layer meshes */
  private meshes: (Mesh | null)[] = []

  /** Committed results */
  private resolvedPaths: ResolvedPath[] = []

  /** Visualization: trace obstacle polygons (with layer info) */
  private traceObstaclePolys: Array<{
    polygon: Point[]
    connectionName: string
    layerZ: number
  }> = []

  /** Current ordering phase */
  private phase: Phase = "shortest"

  /** Best results across all phases */
  private bestResults: ResolvedPath[] | null = null
  private bestObstaclePolys: typeof this.traceObstaclePolys | null = null

  /** Maximum layers allowed by the SRJ */
  private maxLayerCount: number

  /** Points shared by multiple connections (no endcap clearance here) */
  private sharedPoints: Set<string>

  // -----------------------------------------------------------------------
  // Physics relaxation
  // -----------------------------------------------------------------------

  /** Physics relaxer instance (reused across calls) */
  private relaxer: TracePhysicsRelaxer

  /** How many soft Jacobi iterations after each committed trace */
  private static RELAX_ITER_COMMIT = 4
  /** How many soft Jacobi iterations when stuck */
  private static RELAX_ITER_STUCK = 14
  /** Number of stuck-relaxation attempts before advancing the phase */
  private static MAX_STUCK_RELAX = 2

  /**
   * Relaxation state machine.
   * "idle"    – not relaxing; normal routing.
   * "soft"    – running Jacobi soft iterations (one per _step call).
   * "hard"    – about to run the hard depenetration pass + CDT rebuild.
   */
  private relaxState: "idle" | "soft" | "hard" = "idle"
  /** Remaining soft iterations in the current relaxation session. */
  private relaxSoftItersLeft = 0
  /** Whether the current relaxation was triggered by being stuck. */
  private relaxIsStuck = false

  /** Counts how many relaxation attempts have been made in the current phase */
  private stuckRelaxCount = 0

  /** Time tracking for timeout guard */
  private solveStartTime: number = 0
  private static MAX_SOLVE_TIME_MS = 30_000

  /** Total connections for progress tracking */
  private totalConnections: number

  /** Validation results populated after solving */
  validationResult: {
    totalConnections: number
    routedConnections: number
    unroutedConnections: string[]
    crossNetCrossings: Array<{
      traceA: string
      traceB: string
      layer: number
      point: { x: number; y: number }
    }>
  } | null = null

  /** When true, use Mesh.setObstacleBlocked() to toggle obstacle occupancy
   *  instead of rebuilding the CDT per-connection (buildMeshExcluding).
   *  Toggle approach: O(polygons) per connection, no CDT rebuild.
   *  Exclusion approach: O(CDT) per connection, correct but slower. */
  private useOccupancyToggle: boolean

  /** Per-layer: maps connNames key → array of CDT obstacle indices to toggle.
   *  Built once when the global mesh is constructed. */
  private connObstacleIndices: Map<string, number[]>[] = []

  /** Per-layer: maps "x,y" endpoint position → array of CDT obstacle indices
   *  at that position (base obstacles + endpoint octagons).  Used by the
   *  toggle approach to unblock only the specific obstacles at a connection's
   *  start/end, not every obstacle on the same net. */
  private endpointObstacleIndices: Map<string, number[]>[] = []

  constructor(params: {
    srj: SimpleRouteJson
    colorMap: Record<string, string>
    minTraceWidth: number
    margin: number
    useObstacles?: boolean
    usePolyanya?: boolean
    useOccupancyToggle?: boolean
  }) {
    super()
    this.srj = params.srj
    this.colorMap = params.colorMap
    this.minTraceWidth = params.minTraceWidth
    this.margin = params.margin
    this.useObstacles = params.useObstacles ?? true
    this.useOccupancyToggle = params.useOccupancyToggle ?? true
    this.usePolyanya = params.usePolyanya ?? true
    this.maxLayerCount = Math.max(1, params.srj.layerCount ?? 2)
    this.layerCount = Math.min(2, this.maxLayerCount) // Start with 2 layers for layer-aware routing
    this.viaDiameter = params.srj.minViaDiameter ?? 0.6

    // Build layer name → z-index map
    const allLayerNames = this.getAllLayerNames()
    for (let z = 0; z < allLayerNames.length; z++) {
      this.layerNameToZ.set(allLayerNames[z]!, z)
    }

    // Build per-layer base obstacle polygons for ALL possible layers upfront
    // (so layer names remain stable when layerCount grows dynamically).
    // Also record which connection names each obstacle is connected to, so we
    // can exclude them when routing their own traces.
    this.baseObstaclePolygons = []
    this.baseObstacleConnectedTo = []
    for (let z = 0; z < this.maxLayerCount; z++) {
      const layerName = allLayerNames[z]!
      const layerObstacles = params.srj.obstacles.filter((obs) =>
        obs.layers.includes(layerName),
      )
      const expandedPolygons = layerObstacles.map((obs) =>
        rectToPolygon(
          obs.center.x,
          obs.center.y,
          obs.width,
          obs.height,
          this.margin,
        ),
      )
      const connectedToNames = layerObstacles.map((obs) =>
        [...obs.connectedTo],
      )
      this.baseObstaclePolygons.push(expandedPolygons)
      this.baseObstacleConnectedTo.push(
        connectedToNames.map((names) => names.join(",")),
      )
    }

    // Initialize per-layer state (only for active layers)
    this.rectObstacles = this.baseObstaclePolygons
      .slice(0, this.layerCount)
      .map((polys) => [...polys])

    // Add octagonal endpoint obstacles for every connection start/end point.
    // These ensure that pad endpoints are blocked for all OTHER traces, while
    // being excluded (via connectedTo) when routing the owning trace.
    // The octagon is the bounding octagon that fully contains a circle of
    // radius=clearance.  For a regular octagon with circumradius R, the
    // inradius (edge midpoint distance) = R × cos(π/8).  We need inradius ≥
    // clearance, so R = clearance / cos(π/8) ≈ clearance × 1.0824.
    const baseClearance = this.minTraceWidth / 2 + this.margin
    const endpointClearance = baseClearance / Math.cos(Math.PI / 8)
    for (const conn of params.srj.connections) {
      const pts = conn.pointsToConnect
      const connNames = [conn.name]
      if (conn.rootConnectionName && conn.rootConnectionName !== conn.name) {
        connNames.push(conn.rootConnectionName)
      }
      const connStr = connNames.join(",")
      for (const pt of [pts[0]!, pts[pts.length - 1]!]) {
        const layerZ = this.connectionPointToLayerZ(pt)
        if (layerZ >= this.layerCount) continue
        const poly = GreedySequentialPathSolver.makeOctagon(
          pt.x,
          pt.y,
          endpointClearance,
        )
        this.rectObstacles[layerZ]!.push(poly)
        // Track connectedTo for this endpoint obstacle so it gets excluded
        // when routing its own connection
        if (this.baseObstacleConnectedTo[layerZ]) {
          this.baseObstacleConnectedTo[layerZ]!.push(connStr)
        }
        if (this.baseObstaclePolygons[layerZ]) {
          this.baseObstaclePolygons[layerZ]!.push(poly)
        }
      }
    }

    this.baseObstacleCount = this.rectObstacles.map((polys) => polys.length)
    this.connectionMeshCache = Array.from(
      { length: this.layerCount },
      () => new Map(),
    )
    this.tracePolygonObstacles = Array.from(
      { length: this.layerCount },
      () => [],
    )
    this.tracePolyAABBs = Array.from({ length: this.layerCount }, () => [])
    this.tracePolyNetNames = Array.from({ length: this.layerCount }, () => [])
    this.traceWeightedRegions = Array.from(
      { length: this.layerCount },
      () => [],
    )
    this.meshes = Array.from({ length: this.layerCount }, () => null)

    // Build meshes for active layers only
    for (let z = 0; z < this.layerCount; z++) {
      this.buildMesh(z)
    }

    // Detect points shared by multiple connections (junctions — no endcap)
    const pointCounts = new Map<string, number>()
    for (const conn of params.srj.connections) {
      for (const pt of conn.pointsToConnect) {
        const key = `${pt.x},${pt.y}`
        pointCounts.set(key, (pointCounts.get(key) ?? 0) + 1)
      }
    }
    this.sharedPoints = new Set<string>()
    pointCounts.forEach((count, key) => {
      if (count > 1) this.sharedPoints.add(key)
    })

    // Initialize physics relaxer (clearance = same as obstacle polygon clearance)
    const relaxClearance = this.minTraceWidth / 2 + this.margin
    this.relaxer = new TracePhysicsRelaxer(relaxClearance)

    this.allConnections = params.srj.connections.map((conn) => {
      const pts = conn.pointsToConnect
      const originalStart = { x: pts[0]!.x, y: pts[0]!.y }
      const originalEnd = {
        x: pts[pts.length - 1]!.x,
        y: pts[pts.length - 1]!.y,
      }

      // Determine which layer each endpoint is on from the connection point data
      const startLayerZ = this.connectionPointToLayerZ(pts[0]!)
      const endLayerZ = this.connectionPointToLayerZ(pts[pts.length - 1]!)

      // Build all names that might appear in obstacle connectedTo lists:
      // the connection name itself, rootConnectionName, and for merged names
      // like "source_trace_8__source_trace_9", the constituent parts.
      const connNames = [conn.name]
      if (conn.rootConnectionName && conn.rootConnectionName !== conn.name) {
        connNames.push(conn.rootConnectionName)
      }
      if (conn.name.includes("__")) {
        for (const part of conn.name.split("__")) {
          if (!connNames.includes(part)) connNames.push(part)
        }
      }
      const startCandidates = this.getNudgeCandidates(
        originalStart,
        originalEnd,
        params.srj.obstacles,
        connNames,
      )
      const endCandidates = this.getNudgeCandidates(
        originalEnd,
        originalStart,
        params.srj.obstacles,
        connNames,
      )
      return {
        name: conn.name,
        originalStart,
        originalEnd,
        start: startCandidates[0] ?? originalStart,
        end: endCandidates[0] ?? originalEnd,
        startCandidates,
        endCandidates,
        startLayerZ,
        endLayerZ,
        congestionScore: 0, // computed below
        connNames,
      }
    })

    // -----------------------------------------------------------------------
    // Congestion scoring: count how many other pad endpoints and obstacles
    // are near each connection's endpoints.  Traces in crowded areas should
    // be routed first while there is still space available.
    // -----------------------------------------------------------------------
    // Collect all pad positions (every endpoint of every connection)
    const allPads: { x: number; y: number }[] = []
    for (const conn of params.srj.connections) {
      for (const pt of conn.pointsToConnect) {
        allPads.push({ x: pt.x, y: pt.y })
      }
    }
    // Also include obstacle centers as congestion contributors
    for (const obs of params.srj.obstacles) {
      allPads.push({ x: obs.center.x, y: obs.center.y })
    }

    // Congestion radius: nearby pads within this distance contribute.
    // Use a multiple of the margin so the radius scales with board density.
    const congestionRadius = Math.max(this.margin * 6, this.viaDiameter * 4)
    const r2 = congestionRadius * congestionRadius

    for (const conn of this.allConnections) {
      let score = 0
      const sx = conn.originalStart.x
      const sy = conn.originalStart.y
      const ex = conn.originalEnd.x
      const ey = conn.originalEnd.y

      for (const pad of allPads) {
        const dxs = pad.x - sx
        const dys = pad.y - sy
        if (dxs * dxs + dys * dys < r2) score++
        const dxe = pad.x - ex
        const dye = pad.y - ey
        if (dxe * dxe + dye * dye < r2) score++
      }
      // Subtract 2 for self (each endpoint counts itself)
      conn.congestionScore = Math.max(0, score - 2)
    }

    this.remaining = [...this.allConnections]
    this.totalConnections = this.remaining.length
    this.MAX_ITERATIONS = Math.max(500, this.totalConnections * 10)

    // Rebuild endpoint obstacle mappings now that allConnections is populated.
    // The initial buildMesh calls in the constructor ran before allConnections
    // existed, so endpointObstacleIndices was empty.
    if (this.useOccupancyToggle) {
      for (let z = 0; z < this.layerCount; z++) {
        this.rebuildEndpointMapping(z)
      }
    }
  }

  /** Layer names based on maxLayerCount (stable even as layerCount grows) */
  private getAllLayerNames(): string[] {
    const names: string[] = []
    for (let z = 0; z < this.maxLayerCount; z++) {
      if (z === 0) names.push("top")
      else if (z === this.maxLayerCount - 1) names.push("bottom")
      else names.push(`inner${z}`)
    }
    return names
  }

  private getLayerNames(): string[] {
    return this.getAllLayerNames().slice(0, this.layerCount)
  }

  /** Map from layer name → z-index (built once in constructor) */
  private layerNameToZ: Map<string, number> = new Map()

  /** Convert a connection point's layer name(s) to a z-index.
   *  Returns 0 (top) as default. If point is on multiple layers, returns
   *  the first matching z-index (prefers top). */
  private connectionPointToLayerZ(pt: ConnectionPoint): number {
    const layers = getConnectionPointLayers(pt)
    for (const name of layers) {
      const z = this.layerNameToZ.get(name)
      if (z !== undefined) return z
    }
    return 0
  }

  /** Max total obstacle vertices before we skip mesh rebuild (OOM guard) */
  private static MAX_OBSTACLE_VERTICES = 50_000

  private buildMesh(layerZ: number) {
    // Merge overlapping rect obstacles (axis-aligned) to avoid CDT failures
    // from shared/overlapping edges. Trace polygon obstacles are NOT merged
    // since they may not be axis-aligned.
    const mergedRects = mergeOverlappingRects(this.rectObstacles[layerZ]!)
    const allObstacles = [
      ...mergedRects,
      ...this.tracePolygonObstacles[layerZ]!,
    ]

    // Guard: skip rebuild if obstacle complexity is too high
    const totalVerts = allObstacles.reduce((sum, poly) => sum + poly.length, 0)
    if (totalVerts > GreedySequentialPathSolver.MAX_OBSTACLE_VERTICES) {
      console.warn(
        `buildMesh: skipping layer ${layerZ} rebuild — ${totalVerts} vertices exceeds limit`,
      )
      return
    }

    const cdtResult = cdtTriangulate({
      bounds: this.srj.bounds,
      obstacles: allObstacles,
    })
    if (!cdtResult) {
      console.warn(`buildMesh: CDT failed for layer ${layerZ}`)
      return
    }
    const rawMesh = buildMeshFromRegions(cdtResult)
    this.meshes[layerZ] = mergeMesh(rawMesh)

    // Invalidate connection-specific mesh cache (obstacles changed)
    if (this.connectionMeshCache[layerZ]) {
      this.connectionMeshCache[layerZ]!.clear()
    }

    // Build obstacle-index → connNames mapping for occupancy toggle.
    // allObstacles = mergedRects (from base+endpoint+via rects) + tracePolys.
    // We need to know which CDT obstacle indices belong to which connection
    // so we can toggle them with mesh.setObstacleBlocked().
    //
    // Since mergeOverlappingRects can change the mapping from the original
    // rectObstacles, we use the baseObstacleConnectedTo as a lookup:
    // for each obstacle index in the mesh, check if any allObstacles[i]
    // centroid falls inside a known base obstacle polygon.
    if (this.useOccupancyToggle) {
      const map = new Map<string, number[]>()
      const mesh = this.meshes[layerZ]!
      const connectedToList = this.baseObstacleConnectedTo[layerZ]
      const baseCount = this.baseObstacleCount[layerZ] ?? 0
      const rects = this.rectObstacles[layerZ]!

      // Build a mapping from CDT obstacle index → connStr by checking which
      // base obstacle (from the unmerged rectObstacles) each CDT obstacle
      // centroid falls inside.
      const obstIdxToConnStr = new Map<number, string>()
      for (const obsIdx of mesh.getObstacleIndices()) {
        if (obsIdx < 0 || obsIdx >= allObstacles.length) continue
        const poly = allObstacles[obsIdx]!
        // Centroid of the CDT obstacle polygon
        let cx = 0, cy = 0
        for (const p of poly) { cx += p.x; cy += p.y }
        cx /= poly.length; cy /= poly.length

        // Match against unmerged base obstacles using connectedTo
        if (connectedToList) {
          for (let bi = 0; bi < baseCount && bi < connectedToList.length; bi++) {
            const basePoly = rects[bi]!
            if (this.pointInPolygon(cx, cy, basePoly)) {
              obstIdxToConnStr.set(obsIdx, connectedToList[bi]!)
              break
            }
          }
        }
        // Also match trace polygon obstacles by net name
        const tracePolyOffset = mergedRects.length
        if (obsIdx >= tracePolyOffset) {
          const tpIdx = obsIdx - tracePolyOffset
          const traceNets = this.tracePolyNetNames[layerZ]!
          if (tpIdx < traceNets.length) {
            obstIdxToConnStr.set(obsIdx, traceNets[tpIdx]!)
          }
        }
      }

      // Group obstacle indices by connStr
      for (const [obsIdx, connStr] of obstIdxToConnStr) {
        let arr = map.get(connStr)
        if (!arr) { arr = []; map.set(connStr, arr) }
        arr.push(obsIdx)
      }
      this.connObstacleIndices[layerZ] = map

      this.rebuildEndpointMapping(layerZ)
    }
  }

  /** Rebuild endpoint → obstacle index mapping for a layer.
   *  For each obstacle polygon in the mesh, checks which connection
   *  endpoints fall inside it so only those specific obstacles get
   *  unblocked when routing (not the entire net). */
  private rebuildEndpointMapping(layerZ: number): void {
    const mesh = this.meshes[layerZ]
    if (!mesh) return
    const conns = this.allConnections
    if (!conns || conns.length === 0) return

    const epMap = new Map<string, number[]>()
    for (const obsIdx of mesh.getObstacleIndices()) {
      // Check ALL mesh polygons with this obstacleIndex (obstacle may be
      // decomposed into multiple triangles by the CDT).
      for (const poly of mesh.polygons) {
        if (poly.obstacleIndex !== obsIdx) continue
        const verts = poly.vertices.map((vi) => mesh.vertices[vi]!.p)
        for (const conn of conns) {
          if (conn.startLayerZ === layerZ) {
            const key = `${conn.originalStart.x},${conn.originalStart.y}`
            if (this.pointInPolygon(conn.originalStart.x, conn.originalStart.y, verts)) {
              let arr = epMap.get(key)
              if (!arr) { arr = []; epMap.set(key, arr) }
              if (!arr.includes(obsIdx)) arr.push(obsIdx)
            }
          }
          if (conn.endLayerZ === layerZ) {
            const key = `${conn.originalEnd.x},${conn.originalEnd.y}`
            if (this.pointInPolygon(conn.originalEnd.x, conn.originalEnd.y, verts)) {
              let arr = epMap.get(key)
              if (!arr) { arr = []; epMap.set(key, arr) }
              if (!arr.includes(obsIdx)) arr.push(obsIdx)
            }
          }
        }
      }
    }
    this.endpointObstacleIndices[layerZ] = epMap
  }

  /** Simple point-in-polygon test */
  private pointInPolygon(px: number, py: number, poly: Point[]): boolean {
    let inside = false
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const pi = poly[i]!, pj = poly[j]!
      if ((pi.y > py) !== (pj.y > py) &&
          px < (pj.x - pi.x) * (py - pi.y) / (pj.y - pi.y) + pi.x) {
        inside = !inside
      }
    }
    return inside
  }

  /**
   * Toggle obstacle occupancy for a connection on the global mesh.
   * Only unblocks the specific obstacles at the connection's start/end
   * positions — NOT every obstacle on the same net.  This prevents
   * traces from bridging through unrelated pads on the same net.
   *
   * Also toggles same-net trace obstacles (for multi-segment net routing).
   */
  private toggleConnectionObstacles(
    layerZ: number,
    conn: { originalStart: Point; originalEnd: Point; connNames: string[] },
    baseNet: string,
    blocked: boolean,
  ): void {
    const mesh = this.meshes[layerZ]
    if (!mesh) return

    // Toggle only obstacles at this connection's specific endpoints
    const epMap = this.endpointObstacleIndices[layerZ]
    if (epMap) {
      const startKey = `${conn.originalStart.x},${conn.originalStart.y}`
      const endKey = `${conn.originalEnd.x},${conn.originalEnd.y}`
      for (const key of [startKey, endKey]) {
        const indices = epMap.get(key)
        if (indices) {
          for (const idx of indices) mesh.setObstacleBlocked(idx, blocked)
        }
      }
    }

    // Toggle same-net trace obstacles (for multi-segment net routing)
    const connMap = this.connObstacleIndices[layerZ]
    if (connMap) {
      const netIndices = connMap.get(baseNet)
      if (netIndices) {
        for (const idx of netIndices) mesh.setObstacleBlocked(idx, blocked)
      }
    }
  }

  /** Quick check: does this connection have any obstacles that would be
   *  excluded by buildMeshExcluding?  If not, the global mesh is identical
   *  and we can skip the expensive CDT rebuild entirely. */
  private connectionHasExclusions(
    layerZ: number,
    connNames: string[],
    baseNet: string,
  ): boolean {
    // Check base obstacles
    const connectedToList = this.baseObstacleConnectedTo[layerZ]
    const baseCount = this.baseObstacleCount[layerZ] ?? 0
    if (connectedToList) {
      for (let i = 0; i < baseCount; i++) {
        const obsConnStr = connectedToList[i]!
        if (connNames.some((cn) => obsConnStr.includes(cn))) return true
      }
    }
    // Check same-net trace obstacles
    const traceNets = this.tracePolyNetNames[layerZ]!
    for (let i = 0; i < traceNets.length; i++) {
      if (traceNets[i] === baseNet) return true
    }
    return false
  }

  /**
   * Build a CDT mesh for `layerZ` that excludes base obstacle polygons
   * electrically connected to `connNames`, and also excludes same-net trace
   * obstacle polygons (for multi-segment net routing).  Cached per connNames
   * key so repeated lookups in pickBestOnLayer are cheap.
   */
  private buildMeshExcluding(
    layerZ: number,
    connNames: string[],
  ): Mesh | null {
    const cacheKey = connNames.join("|")
    const cache = this.connectionMeshCache[layerZ]
    if (cache?.has(cacheKey)) return cache.get(cacheKey) ?? null

    const baseCount = this.baseObstacleCount[layerZ] ?? 0
    const connectedToList = this.baseObstacleConnectedTo[layerZ]
    const rects = this.rectObstacles[layerZ]!

    // Filter base obstacle polygons: keep only those NOT connected to connNames
    const filteredRects: Point[][] = []
    for (let i = 0; i < rects.length; i++) {
      if (i < baseCount && connectedToList) {
        // This is a base obstacle — check if any connName appears in its connectedTo
        const obsConnStr = connectedToList[i]!
        const exclude = connNames.some((cn) => obsConnStr.includes(cn))
        if (exclude) continue
      }
      filteredRects.push(rects[i]!)
    }

    // Filter trace polygon obstacles: exclude same-net traces (for multi-segment routing)
    const baseNet = connNames.length > 0 ? connNames[0]! : ""
    const filteredTracePolys: Point[][] = []
    const tracePolys = this.tracePolygonObstacles[layerZ]!
    const traceNets = this.tracePolyNetNames[layerZ]!
    for (let i = 0; i < tracePolys.length; i++) {
      // Keep if different net
      if (traceNets[i] !== GreedySequentialPathSolver.baseNetName(baseNet)) {
        filteredTracePolys.push(tracePolys[i]!)
      }
    }

    const mergedRects = mergeOverlappingRects(filteredRects)
    const allObstacles = [...mergedRects, ...filteredTracePolys]

    const totalVerts = allObstacles.reduce((sum, poly) => sum + poly.length, 0)
    if (totalVerts > GreedySequentialPathSolver.MAX_OBSTACLE_VERTICES) {
      cache?.set(cacheKey, null)
      return null
    }

    const cdtResult = cdtTriangulate({
      bounds: this.srj.bounds,
      obstacles: allObstacles,
    })
    if (!cdtResult) {
      cache?.set(cacheKey, null)
      return null
    }
    const rawMesh = buildMeshFromRegions(cdtResult)
    const mesh = mergeMesh(rawMesh)
    cache?.set(cacheKey, mesh)
    return mesh
  }

  /**
   * If a point sits on/inside expanded obstacles that it's connected to,
   * nudge it outward so it clears all expanded boundaries. Uses the
   * direction toward the other endpoint to pick the escape side.
   * Lands slightly beyond the edge so Polyanya doesn't treat it as inside.
   */
  private nudgeOutOfObstacle(
    pt: { x: number; y: number },
    other: { x: number; y: number },
    obstacles: SimpleRouteJson["obstacles"],
    connNames: string[],
  ): Point {
    const ranked = this.getNudgeCandidates(pt, other, obstacles, connNames)
    return ranked[0] ?? { x: pt.x, y: pt.y }
  }

  /**
   * Returns up to 4 nudge candidates ranked by alignment toward the other
   * endpoint, then by shortest escape distance. The first element is the
   * "best" direction; callers can try others if routing fails.
   */
  private getNudgeCandidates(
    pt: { x: number; y: number },
    other: { x: number; y: number },
    obstacles: SimpleRouteJson["obstacles"],
    connNames: string[],
  ): Point[] {
    const connected = obstacles.filter((obs) =>
      connNames.some((n) => obs.connectedTo.includes(n)),
    )
    if (connected.length === 0) return [{ x: pt.x, y: pt.y }]

    // Find the innermost connected obstacle that contains the point
    let containingObs: (typeof connected)[0] | null = null
    for (const obs of connected) {
      const halfW = obs.width / 2 + this.margin + 0.05
      const halfH = obs.height / 2 + this.margin + 0.05
      if (
        Math.abs(pt.x - obs.center.x) < halfW &&
        Math.abs(pt.y - obs.center.y) < halfH
      ) {
        // Prefer smaller (more specific) obstacle
        if (
          !containingObs ||
          obs.width * obs.height < containingObs.width * containingObs.height
        ) {
          containingObs = obs
        }
      }
    }
    if (!containingObs) return [{ x: pt.x, y: pt.y }]

    const obs = containingObs
    const halfW = obs.width / 2 + this.margin + 0.05
    const halfH = obs.height / 2 + this.margin + 0.05
    const toOtherX = other.x - pt.x
    const toOtherY = other.y - pt.y
    const dx = pt.x - obs.center.x
    const dy = pt.y - obs.center.y

    const rawCandidates = [
      { x: obs.center.x + halfW, y: pt.y, score: toOtherX, dist: halfW - dx },
      { x: obs.center.x - halfW, y: pt.y, score: -toOtherX, dist: halfW + dx },
      { x: pt.x, y: obs.center.y + halfH, score: toOtherY, dist: halfH - dy },
      { x: pt.x, y: obs.center.y - halfH, score: -toOtherY, dist: halfH + dy },
    ]
    rawCandidates.sort((a, b) => {
      const aAligned = a.score > 0 ? 1 : 0
      const bAligned = b.score > 0 ? 1 : 0
      if (aAligned !== bAligned) return bAligned - aAligned
      return a.dist - b.dist
    })

    // For each raw candidate, iteratively push out of any remaining connected obstacles
    const results: Point[] = []
    for (const cand of rawCandidates) {
      let result = { x: cand.x, y: cand.y }
      for (let iter = 0; iter < 5; iter++) {
        let pushed = false
        for (const obs2 of connected) {
          const hw = obs2.width / 2 + this.margin + 0.05
          const hh = obs2.height / 2 + this.margin + 0.05
          const d2x = result.x - obs2.center.x
          const d2y = result.y - obs2.center.y
          if (Math.abs(d2x) < hw && Math.abs(d2y) < hh) {
            // Push same direction as the initial escape
            const distRight = hw - d2x
            const distLeft = hw + d2x
            const distTop = hh - d2y
            const distBottom = hh + d2y
            const minDist = Math.min(distRight, distLeft, distTop, distBottom)
            if (minDist === distRight)
              result = { x: obs2.center.x + hw, y: result.y }
            else if (minDist === distLeft)
              result = { x: obs2.center.x - hw, y: result.y }
            else if (minDist === distBottom)
              result = { x: result.x, y: obs2.center.y - hh }
            else result = { x: result.x, y: obs2.center.y + hh }
            pushed = true
          }
        }
        if (!pushed) break
      }
      results.push(result)
    }

    return results
  }

  // -----------------------------------------------------------------------
  // Physics relaxation helpers
  // -----------------------------------------------------------------------

  /** Net-name function used by the relaxer for same-net exemption */
  private netNameForRelaxer(connectionName: string): string {
    return GreedySequentialPathSolver.baseNetName(connectionName)
  }

  /**
   * Rebuild all trace polygon obstacles from the current resolvedPaths vertex
   * positions and then rebuild CDT meshes for all active layers.
   *
   * Call this after modifying route vertex positions (e.g. after relaxation).
   * Via obstacles (stored in rectObstacles) are not touched.
   */
  private rebuildAllTraceObstacles() {
    // Clear trace polygon obstacles only
    for (let z = 0; z < this.layerCount; z++) {
      this.tracePolygonObstacles[z] = []
      this.tracePolyAABBs[z] = []
      this.tracePolyNetNames[z] = []
    }
    this.traceObstaclePolys = []

    const clearance = this.minTraceWidth / 2 + this.margin
    const connMap = new Map(this.allConnections.map((c) => [c.name, c]))

    for (const rp of this.resolvedPaths) {
      const connInfo = connMap.get(rp.connectionName)

      // Walk through the route and extract runs of same-layer points
      let runStart = 0
      for (let i = 1; i <= rp.route.length; i++) {
        const prevZ = rp.route[i - 1]!.z
        const curZ = i < rp.route.length ? rp.route[i]!.z : -1

        if (curZ !== prevZ || i === rp.route.length) {
          const run = rp.route.slice(runStart, i)
          const layerZ = prevZ

          if (run.length >= 2 && layerZ < this.layerCount) {
            const pts = run.map((p) => ({ x: p.x, y: p.y }))
            // Pass original endpoints only for the first/last run so endcap
            // extension logic in pathToObstaclePolygons works correctly
            const isFirstRun = runStart === 0
            const isLastRun = i === rp.route.length
            const polys = this.pathToObstaclePolygons(
              pts,
              clearance,
              isFirstRun ? connInfo?.originalStart : undefined,
              isLastRun ? connInfo?.originalEnd : undefined,
            )
            for (const poly of polys) {
              this.pushTraceObstacle(layerZ, poly, rp.connectionName)
              this.traceObstaclePolys.push({
                polygon: poly,
                connectionName: rp.connectionName,
                layerZ,
              })
            }
          }
          runStart = i
        }
      }
    }

    // Rebuild CDT meshes for all active layers
    for (let z = 0; z < this.layerCount; z++) {
      this.buildMesh(z)
    }
  }

  /** Lazily built list of relaxer obstacles (one per SRJ obstacle, per active layer). */
  private relaxerObstacles: RelaxerObstacle[] | null = null

  private buildRelaxerObstacles(): RelaxerObstacle[] {
    const allLayerNames = this.getAllLayerNames()
    return this.srj.obstacles.map((obs) => {
      const layers = obs.layers
        .map((name) => allLayerNames.indexOf(name))
        .filter((z) => z >= 0)
      return {
        layers,
        cx: obs.center.x,
        cy: obs.center.y,
        hw: obs.width / 2 + this.margin,
        hh: obs.height / 2 + this.margin,
      }
    })
  }

  private buildFixedKeys(): Set<string> {
    const keys = new Set<string>()
    for (const conn of this.allConnections) {
      keys.add(`${conn.originalStart.x},${conn.originalStart.y}`)
      keys.add(`${conn.originalEnd.x},${conn.originalEnd.y}`)
    }
    return keys
  }

  /**
   * Begin a relaxation session: enter "soft" state.
   * Each subsequent _step() call will run one Jacobi iteration (visible in
   * the debugger) until the budget is exhausted, then a hard pass runs and
   * the CDT is rebuilt.
   */
  private beginRelax(softIters: number, isStuck: boolean): void {
    if (!this.useObstacles || this.resolvedPaths.length === 0) return
    if (!this.relaxerObstacles)
      this.relaxerObstacles = this.buildRelaxerObstacles()

    this.relaxer.startSession(
      this.resolvedPaths,
      this.buildFixedKeys(),
      (name) => this.netNameForRelaxer(name),
      this.relaxerObstacles,
      0.2, // strong string-pull keeps subdivided traces coherent
    )
    this.relaxSoftItersLeft = softIters
    this.relaxIsStuck = isStuck
    this.relaxState = "soft"
  }

  /** Drive one step of the active relaxation session. Returns true if still relaxing. */
  private stepRelax(): boolean {
    if (this.relaxState === "idle") return false

    if (this.relaxState === "soft") {
      const moved = this.relaxer.softStep()
      this.relaxSoftItersLeft--
      // Stop early if nothing moved; always run at least 1 iteration
      if (this.relaxSoftItersLeft <= 0 || !moved) {
        this.relaxState = "hard"
      }
      return true // still in relaxation
    }

    if (this.relaxState === "hard") {
      // Hard depenetration pass (snaps fixed vertices, projects out of obstacles)
      this.relaxer.hardPass()
      // Rebuild CDT from the freshly relaxed positions
      this.rebuildAllTraceObstacles()
      this.relaxState = "idle"
      return false // relaxation finished
    }

    return false
  }

  /**
   * Insert intermediate vertices along each same-layer segment so the physics
   * relaxer has roughly one vertex per grid-cell length.  This lets the route
   * deform smoothly rather than bending only at the original pathfinder knees.
   *
   * Layer-transition edges (via hops) are never subdivided.
   */
  private subdivideRoute(
    route: { x: number; y: number; z: number }[],
  ): { x: number; y: number; z: number }[] {
    if (route.length < 2) return route
    // Target: one vertex per ~cell-size (= 2 × hardClearance)
    const maxLen = (this.minTraceWidth / 2 + this.margin) * 2
    const out: { x: number; y: number; z: number }[] = [route[0]!]
    for (let i = 1; i < route.length; i++) {
      const a = route[i - 1]!
      const b = route[i]!
      // Never subdivide layer transitions (z changes)
      if (a.z !== b.z) {
        out.push(b)
        continue
      }
      const dx = b.x - a.x
      const dy = b.y - a.y
      const dist = Math.hypot(dx, dy)
      const n = Math.max(1, Math.ceil(dist / maxLen))
      for (let j = 1; j <= n; j++) {
        const t = j / n
        out.push({ x: a.x + dx * t, y: a.y + dy * t, z: b.z })
      }
    }
    return out
  }

  /** Reset routing state back to initial (no committed traces) */
  private resetState() {
    this.stuckRelaxCount = 0
    this.relaxState = "idle"
    this.rectObstacles = this.baseObstaclePolygons
      .slice(0, this.layerCount)
      .map((polys) => [...polys])
    this.tracePolygonObstacles = Array.from(
      { length: this.layerCount },
      () => [],
    )
    this.tracePolyAABBs = Array.from({ length: this.layerCount }, () => [])
    this.tracePolyNetNames = Array.from({ length: this.layerCount }, () => [])
    this.traceWeightedRegions = Array.from(
      { length: this.layerCount },
      () => [],
    )
    this.meshes = Array.from({ length: this.layerCount }, () => null)
    this.connectionMeshCache = Array.from(
      { length: this.layerCount },
      () => new Map(),
    )
    this.connObstacleIndices = Array.from(
      { length: this.layerCount },
      () => new Map(),
    )
    this.endpointObstacleIndices = Array.from(
      { length: this.layerCount },
      () => new Map(),
    )
    this.baseObstacleCount = this.rectObstacles.map((polys) => polys.length)
    this.resolvedPaths = []
    this.traceObstaclePolys = []
    this.remaining = [...this.allConnections]
    for (let z = 0; z < this.layerCount; z++) {
      this.buildMesh(z)
    }
  }

  /**
   * Convert a polyline path into weighted region polygons — many small squares
   * placed along each segment so diagonal segments get tight-fitting coverage.
   */
  private pathToWeightedRegions(
    path: Point[],
    clearance: number,
  ): WeightedRegion[] {
    const regions: WeightedRegion[] = []
    const side = clearance * 2

    for (let i = 0; i < path.length - 1; i++) {
      const a = path[i]!
      const b = path[i + 1]!
      const dx = b.x - a.x
      const dy = b.y - a.y
      const segLen = Math.hypot(dx, dy)

      if (segLen < 1e-9) continue

      const steps = Math.max(1, Math.ceil(segLen / side))
      for (let s = 0; s <= steps; s++) {
        const t = s / steps
        const cx = a.x + dx * t
        const cy = a.y + dy * t
        regions.push({
          polygon: rectToPolygon(cx, cy, side, side, 0),
          weight: TRACE_WEIGHT,
          penalty: TRACE_PENALTY,
        })
      }
    }

    return regions
  }

  /**
   * Convert a polyline path into a single thick polygon obstacle by offsetting
   * each segment perpendicular to its direction. Produces a tight-fitting
   * outline that follows diagonal segments precisely, unlike axis-aligned rects.
   *
   * Returns an array with one polygon (or empty if path is degenerate).
   */
  private pathToObstaclePolygons(
    path: Point[],
    clearance: number,
    originalStart?: Point,
    originalEnd?: Point,
  ): Point[][] {
    if (path.length < 2) return []

    // Deduplicate consecutive coincident points
    const pts: Point[] = [path[0]!]
    for (let i = 1; i < path.length; i++) {
      const prev = pts[pts.length - 1]!
      const cur = path[i]!
      if (Math.abs(cur.x - prev.x) > 1e-9 || Math.abs(cur.y - prev.y) > 1e-9) {
        pts.push(cur)
      }
    }
    if (pts.length < 2) return []

    // No linear endpoint extension needed — the semicircular endcaps added
    // at the end of this method already provide clearance coverage beyond
    // each endpoint.  The old linear extension was redundant and caused
    // double-extension (2× clearance) at the trace tips.

    // Compute per-segment unit normals (pointing left when walking A→B)
    const normals: { nx: number; ny: number }[] = []
    for (let i = 0; i < pts.length - 1; i++) {
      const dx = pts[i + 1]!.x - pts[i]!.x
      const dy = pts[i + 1]!.y - pts[i]!.y
      const len = Math.hypot(dx, dy)
      // Normal rotated 90° CCW: (-dy, dx)
      normals.push({ nx: -dy / len, ny: dx / len })
    }

    // Build left side (forward along path, offset +clearance in normal dir)
    // and right side (offset -clearance). At interior vertices, use a bevel
    // join on the outside of each turn so the polygon covers full corners.
    const left: Point[] = []
    const right: Point[] = []

    for (let i = 0; i < pts.length; i++) {
      const p = pts[i]!

      if (i === 0) {
        const n = normals[0]!
        left.push({ x: p.x + n.nx * clearance, y: p.y + n.ny * clearance })
        right.push({ x: p.x - n.nx * clearance, y: p.y - n.ny * clearance })
      } else if (i === pts.length - 1) {
        const n = normals[normals.length - 1]!
        left.push({ x: p.x + n.nx * clearance, y: p.y + n.ny * clearance })
        right.push({ x: p.x - n.nx * clearance, y: p.y - n.ny * clearance })
      } else {
        const n0 = normals[i - 1]!
        const n1 = normals[i]!

        const d0x = pts[i]!.x - pts[i - 1]!.x
        const d0y = pts[i]!.y - pts[i - 1]!.y
        const d1x = pts[i + 1]!.x - pts[i]!.x
        const d1y = pts[i + 1]!.y - pts[i]!.y
        const cross = d0x * d1y - d0y * d1x

        // Miter normal for the inside of the turn
        let mx = n0.nx + n1.nx
        let my = n0.ny + n1.ny
        const mlen = Math.hypot(mx, my)
        if (mlen > 1e-9) {
          const dot = n0.nx * n1.nx + n0.ny * n1.ny
          const cosHalf = Math.sqrt((1 + dot) / 2)
          const scale = cosHalf > 0.3 ? 1 / cosHalf : 1 / 0.3
          mx = (mx / mlen) * scale
          my = (my / mlen) * scale
        } else {
          mx = n0.nx
          my = n0.ny
        }

        if (cross > 1e-9) {
          // Left turn: inside is left (miter), outside is right (bevel)
          left.push({ x: p.x + mx * clearance, y: p.y + my * clearance })
          right.push({ x: p.x - n0.nx * clearance, y: p.y - n0.ny * clearance })
          right.push({ x: p.x - n1.nx * clearance, y: p.y - n1.ny * clearance })
        } else if (cross < -1e-9) {
          // Right turn: inside is right (miter), outside is left (bevel)
          left.push({ x: p.x + n0.nx * clearance, y: p.y + n0.ny * clearance })
          left.push({ x: p.x + n1.nx * clearance, y: p.y + n1.ny * clearance })
          right.push({ x: p.x - mx * clearance, y: p.y - my * clearance })
        } else {
          left.push({ x: p.x + mx * clearance, y: p.y + my * clearance })
          right.push({ x: p.x - mx * clearance, y: p.y - my * clearance })
        }
      }
    }

    // Form the main polygon with flat endcaps (left forward, right reversed).
    const polygon: Point[] = [...left, ...right.reverse()]

    // Add full-circle (octagonal) endcap polygons as separate obstacles at
    // both endpoints.  Using full closed octagons (not semicircular arcs
    // seamed into the main polygon) eliminates floating-point gaps at the
    // seam when obstacles are merged by mergeOverlappingRects.
    const result: Point[][] = [polygon]
    const firstPt = pts[0]!
    const lastPt = pts[pts.length - 1]!
    result.push(GreedySequentialPathSolver.makeOctagon(firstPt.x, firstPt.y, clearance))
    result.push(GreedySequentialPathSolver.makeOctagon(lastPt.x, lastPt.y, clearance))

    return result
  }

  /**
   * Create via obstacle polygons at a point (circle approximated as expanded rect).
   * Vias block both layers so they get added to all layer meshes.
   */
  private viaToObstaclePolygon(viaPoint: Point): Point[] {
    const r = this.viaDiameter / 2
    return rectToPolygon(viaPoint.x, viaPoint.y, r * 2, r * 2, this.margin)
  }

  private static polyAABB(poly: Point[]): {
    minX: number
    minY: number
    maxX: number
    maxY: number
  } {
    let minX = Infinity,
      minY = Infinity,
      maxX = -Infinity,
      maxY = -Infinity
    for (const p of poly) {
      if (p.x < minX) minX = p.x
      if (p.y < minY) minY = p.y
      if (p.x > maxX) maxX = p.x
      if (p.y > maxY) maxY = p.y
    }
    return { minX, minY, maxX, maxY }
  }

  /** Extract base net name: "source_net_3_mst0" → "source_net_3" */
  /** 8-sided approximation of a circle — cheap for CDT but blocks the area. */
  private static makeOctagon(cx: number, cy: number, r: number): Point[] {
    const pts: Point[] = []
    for (let k = 0; k < 8; k++) {
      const angle = (Math.PI * 2 * k) / 8
      pts.push({ x: cx + Math.cos(angle) * r, y: cy + Math.sin(angle) * r })
    }
    return pts
  }

  private static baseNetName(connectionName: string): string {
    const m = connectionName.match(/^(.+?)_mst\d+$/)
    return m ? m[1]! : connectionName
  }

  private pushTraceObstacle(
    layerZ: number,
    poly: Point[],
    connectionName: string,
  ) {
    this.tracePolygonObstacles[layerZ]!.push(poly)
    this.tracePolyAABBs[layerZ]!.push(GreedySequentialPathSolver.polyAABB(poly))
    this.tracePolyNetNames[layerZ]!.push(
      GreedySequentialPathSolver.baseNetName(connectionName),
    )
  }

  /**
   * Check if placing a via at the given point would collide with any existing
   * trace obstacle polygon on any layer. Trace polygons already include
   * clearance, so we only need to check the via's own radius.
   */
  private isViaSafe(point: Point, exemptNet?: string): boolean {
    const viaR = this.viaDiameter / 2
    const vMinX = point.x - viaR
    const vMaxX = point.x + viaR
    const vMinY = point.y - viaR
    const vMaxY = point.y + viaR

    for (let z = 0; z < this.layerCount; z++) {
      const aabbs = this.tracePolyAABBs[z]!
      const nets = this.tracePolyNetNames[z]!
      for (let i = 0; i < aabbs.length; i++) {
        if (exemptNet && nets[i] === exemptNet) continue
        const bb = aabbs[i]!
        if (
          vMaxX > bb.minX &&
          vMinX < bb.maxX &&
          vMaxY > bb.minY &&
          vMinY < bb.maxY
        ) {
          return false
        }
      }
    }
    return true
  }

  /**
   * Check if a circle (via footprint) overlaps a polygon. Returns the
   * minimum separation vector to push the circle out, or null if no overlap.
   * Uses point-in-polygon + closest edge distance for precise collision.
   */
  private circlePolygonOverlap(
    cx: number,
    cy: number,
    radius: number,
    poly: Point[],
  ): { dx: number; dy: number; depth: number } | null {
    // Find the closest point on any polygon edge to the circle center
    let minDist = Infinity
    let closestX = 0
    let closestY = 0
    const n = poly.length

    for (let i = 0; i < n; i++) {
      const a = poly[i]!
      const b = poly[(i + 1) % n]!
      const ex = b.x - a.x
      const ey = b.y - a.y
      const len2 = ex * ex + ey * ey
      if (len2 < 1e-12) continue
      const t = Math.max(
        0,
        Math.min(1, ((cx - a.x) * ex + (cy - a.y) * ey) / len2),
      )
      const px = a.x + t * ex
      const py = a.y + t * ey
      const d = Math.hypot(cx - px, cy - py)
      if (d < minDist) {
        minDist = d
        closestX = px
        closestY = py
      }
    }

    if (minDist >= radius) return null

    // Circle center might be inside the polygon — check with ray casting
    let inside = false
    for (let i = 0, j = n - 1; i < n; j = i++) {
      const yi = poly[i]!.y,
        yj = poly[j]!.y
      if (yi > cy !== yj > cy) {
        const xi = poly[i]!.x,
          xj = poly[j]!.x
        if (cx < ((xj - xi) * (cy - yi)) / (yj - yi) + xi) {
          inside = !inside
        }
      }
    }

    if (inside) {
      // Center is inside polygon — push toward closest edge
      const dx = cx - closestX
      const dy = cy - closestY
      const d = Math.hypot(dx, dy)
      if (d < 1e-9) {
        // Exactly on the edge — pick arbitrary perpendicular
        return { dx: 1, dy: 0, depth: radius }
      }
      return { dx: dx / d, dy: dy / d, depth: radius + d }
    } else {
      // Center is outside but within radius of an edge
      const dx = cx - closestX
      const dy = cy - closestY
      if (minDist < 1e-9) return { dx: 1, dy: 0, depth: radius }
      return { dx: dx / minDist, dy: dy / minDist, depth: radius - minDist }
    }
  }

  /**
   * Try to find a via-safe point near the given candidate by iteratively
   * pushing it away from colliding trace obstacle polygons. Uses precise
   * circle-polygon collision to get accurate separation vectors.
   * Returns the adjusted point if successful, or null if no safe position
   * found within maxDrift distance of the original candidate.
   */
  private findViaSafePoint(
    candidate: Point,
    maxDrift: number,
    exemptNet?: string,
  ): Point | null {
    const radius = this.viaDiameter / 2
    let px = candidate.x
    let py = candidate.y

    for (let iter = 0; iter < 10; iter++) {
      let pushed = false
      for (let z = 0; z < this.layerCount; z++) {
        const polys = this.tracePolygonObstacles[z]!
        const aabbs = this.tracePolyAABBs[z]!
        const nets = this.tracePolyNetNames[z]!
        for (let i = 0; i < polys.length; i++) {
          if (exemptNet && nets[i] === exemptNet) continue
          const bb = aabbs[i]!
          if (
            px + radius < bb.minX ||
            px - radius > bb.maxX ||
            py + radius < bb.minY ||
            py - radius > bb.maxY
          )
            continue
          const overlap = this.circlePolygonOverlap(px, py, radius, polys[i]!)
          if (overlap) {
            const eps = 0.02
            px += overlap.dx * (overlap.depth + eps)
            py += overlap.dy * (overlap.depth + eps)
            pushed = true
          }
        }
      }
      if (!pushed) {
        const drift = Math.hypot(px - candidate.x, py - candidate.y)
        if (drift <= maxDrift) return { x: px, y: py }
        return null
      }
    }

    // After iterations, final check
    if (this.isViaSafe({ x: px, y: py }, exemptNet)) {
      const drift = Math.hypot(px - candidate.x, py - candidate.y)
      if (drift <= maxDrift) return { x: px, y: py }
    }
    return null
  }

  /**
   * Check if a bridge segment (from original endpoint to nudged endpoint)
   * crosses any already-committed trace segment on the given layer.
   */
  private bridgeCrossesExistingTrace(
    from: Point,
    to: Point,
    layerZ: number,
    exemptNet?: string,
  ): boolean {
    // Skip if bridge is zero-length
    if (Math.abs(from.x - to.x) < 1e-9 && Math.abs(from.y - to.y) < 1e-9)
      return false

    for (const rp of this.resolvedPaths) {
      if (
        exemptNet &&
        GreedySequentialPathSolver.baseNetName(rp.connectionName) === exemptNet
      )
        continue
      for (let i = 0; i < rp.route.length - 1; i++) {
        const a = rp.route[i]!
        const b = rp.route[i + 1]!
        if (a.z !== layerZ || b.z !== layerZ) continue
        if (
          segSegIntersection(from.x, from.y, to.x, to.y, a.x, a.y, b.x, b.y)
        ) {
          return true
        }
      }
    }
    return false
  }

  /** Search using Polyanya SearchInstance (mesh-based, no weighted regions) */
  private searchPolyanya(
    mesh: Mesh,
    start: Point,
    end: Point,
  ): { cost: number; path: Point[] } {
    const si = new SearchInstance(mesh)
    si.setStartGoal(start, end)
    const found = si.search()
    if (!found) return { cost: -1, path: [] }
    return { cost: si.getCost(), path: si.getPathPoints() }
  }

  /** Search using VisibilityGraph (supports weighted regions) */
  private searchVG(
    mesh: Mesh,
    layerZ: number,
    start: Point,
    end: Point,
  ): { cost: number; path: Point[] } {
    const vg = new VisibilityGraph(mesh, {
      weightedRegions: this.useObstacles
        ? []
        : this.traceWeightedRegions[layerZ]!,
    })
    const r = vg.search(start, end)
    return { cost: r.cost, path: r.path }
  }

  /**
   * Pick the best connection to route on a specific layer.
   *
   * Two-phase approach to minimize expensive CDT rebuilds:
   *
   * Phase 1 (cheap): Rank all remaining connections using nudge endpoints
   * on the GLOBAL mesh.  This gives approximate path costs without any CDT
   * rebuilds.  Connections that can't even nudge-route are deprioritized.
   *
   * Phase 2 (expensive): For the top candidates from Phase 1, build the
   * connection-specific exclusion mesh and pathfind from originalStart to
   * originalEnd (the correct, no-nudge path).  Commit the first one that
   * succeeds.
   *
   * Total CDT builds per step: O(1) best case, O(K) worst case.
   */
  private pickBestOnLayer(
    layerZ: number,
    pickShortest: boolean,
  ): { idx: number; path: Point[] } {
    const mesh = this.meshes[layerZ]
    if (!mesh) return { idx: -1, path: [] }

    // -------------------------------------------------------------------
    // Phase 1: Cheap ranking — nudge + global mesh (no CDT rebuild)
    // -------------------------------------------------------------------
    const self = this
    type Candidate = {
      idx: number
      conn: (typeof self.remaining)[0]
      approxCost: number
      routable: boolean
      // For via fallback
      effectiveS?: Point
      effectiveE?: Point
      needsVia: boolean
    }
    const ranked: Candidate[] = []

    for (let i = 0; i < this.remaining.length; i++) {
      const c = this.remaining[i]!
      const needStartVia = c.startLayerZ !== layerZ
      const needEndVia = c.endLayerZ !== layerZ
      const needsVia = needStartVia || needEndVia
      const baseNet = GreedySequentialPathSolver.baseNetName(c.name)

      if (!needsVia) {
        let bestApprox = pickShortest ? Infinity : -Infinity
        let found = false

        if (this.useOccupancyToggle) {
          // Toggle approach: unblock own obstacles, search on global mesh
          this.toggleConnectionObstacles(layerZ, c, baseNet, false)
          const r = this.searchPolyanya(mesh, c.originalStart, c.originalEnd)
          this.toggleConnectionObstacles(layerZ, c, baseNet, true)
          if (r.cost >= 0 && r.path.length > 0) {
            bestApprox = r.cost
            found = true
          }
        } else {
          // Exclusion approach: nudge on global mesh for approximate cost
          const starts =
            c.startCandidates.length > 0 ? c.startCandidates : [c.start]
          const ends = c.endCandidates.length > 0 ? c.endCandidates : [c.end]
          for (const s of starts) {
            for (const e of ends) {
              const r = this.searchPolyanya(mesh, s, e)
              if (r.cost < 0 || r.path.length === 0) continue
              const better = pickShortest
                ? r.cost < bestApprox
                : r.cost > bestApprox
              if (better) bestApprox = r.cost
              found = true
            break
          }
          if (found) break
        }
        } // end exclusion approach
        if (!found) {
          bestApprox = Math.hypot(
            c.originalEnd.x - c.originalStart.x,
            c.originalEnd.y - c.originalStart.y,
          )
        }
        ranked.push({ idx: i, conn: c, approxCost: bestApprox, routable: found, needsVia })
      } else {
        // Via case: compute approximate cost from nudge candidates
        const starts =
          c.startCandidates.length > 0 ? c.startCandidates : [c.start]
        const ends = c.endCandidates.length > 0 ? c.endCandidates : [c.end]
        let bestApprox = pickShortest ? Infinity : -Infinity
        let found = false
        let bestS: Point | undefined
        let bestE: Point | undefined
        const maxViaDrift = this.viaDiameter * 2

        for (const s of starts) {
          for (const e of ends) {
            let effectiveS = s
            let effectiveE = e
            if (needStartVia && !this.isViaSafe(s, baseNet)) {
              const safe = this.findViaSafePoint(s, maxViaDrift, baseNet)
              if (!safe) continue
              effectiveS = safe
            }
            if (needEndVia && !this.isViaSafe(e, baseNet)) {
              const safe = this.findViaSafePoint(e, maxViaDrift, baseNet)
              if (!safe) continue
              effectiveE = safe
            }
            if (
              this.bridgeCrossesExistingTrace(c.originalStart, effectiveS, c.startLayerZ, baseNet) ||
              this.bridgeCrossesExistingTrace(effectiveE, c.originalEnd, c.endLayerZ, baseNet)
            ) continue

            const r = this.usePolyanya
              ? this.searchPolyanya(mesh, effectiveS, effectiveE)
              : this.searchVG(mesh, layerZ, effectiveS, effectiveE)
            if (r.cost < 0 || r.path.length === 0) continue
            const better = pickShortest ? r.cost < bestApprox : r.cost > bestApprox
            if (better) {
              bestApprox = r.cost
              bestS = effectiveS
              bestE = effectiveE
            }
            found = true
            break
          }
          if (found) break
        }
        if (!found) {
          bestApprox = Math.hypot(
            c.originalEnd.x - c.originalStart.x,
            c.originalEnd.y - c.originalStart.y,
          )
        }
        ranked.push({
          idx: i, conn: c, approxCost: bestApprox, routable: found,
          needsVia, effectiveS: bestS, effectiveE: bestE,
        })
      }
    }

    // Sort: routable first, then by approximate cost
    ranked.sort((a, b) => {
      if (a.routable !== b.routable) return a.routable ? -1 : 1
      return pickShortest
        ? a.approxCost - b.approxCost
        : b.approxCost - a.approxCost
    })

    // -------------------------------------------------------------------
    // Phase 2: Commit the best routable candidate.
    // Toggle approach: unblock own obstacles, search, re-block. No CDT rebuild.
    // Exclusion approach: buildMeshExcluding CDT rebuild for the top candidate.
    //
    // After finding a path, validate it doesn't cross any committed trace
    // on the same layer before returning.  This catches CDT leaks where
    // the pathfinder routes through free-space gaps in obstacle polygons.
    // -------------------------------------------------------------------
    // Helper: check if a candidate path crosses any committed trace on this layer
    const pathCrosses = (path: Point[], exemptNet: string): boolean => {
      for (let i = 0; i < path.length - 1; i++) {
        const a = path[i]!
        const b = path[i + 1]!
        for (const rp of this.resolvedPaths) {
          if (GreedySequentialPathSolver.baseNetName(rp.connectionName) === exemptNet) continue
          for (let j = 0; j < rp.route.length - 1; j++) {
            const c = rp.route[j]!
            const d = rp.route[j + 1]!
            if (c.z !== layerZ || d.z !== layerZ) continue
            if (segSegIntersection(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y)) {
              return true
            }
          }
        }
      }
      return false
    }

    for (const cand of ranked) {
      if (!cand.routable) continue
      const c = cand.conn
      const baseNet = GreedySequentialPathSolver.baseNetName(c.name)

      if (!cand.needsVia) {
        let r: { cost: number; path: Point[] } = { cost: -1, path: [] }
        if (this.useOccupancyToggle) {
          this.toggleConnectionObstacles(layerZ, c, baseNet, false)
          r = this.searchPolyanya(mesh, c.originalStart, c.originalEnd)
          this.toggleConnectionObstacles(layerZ, c, baseNet, true)
        } else {
          const connMesh = this.buildMeshExcluding(layerZ, c.connNames)
          if (connMesh) r = this.searchPolyanya(connMesh, c.originalStart, c.originalEnd)
        }
        // If toggle path crosses committed traces, fallback to CDT exclusion
        // (different mesh topology may provide a crossing-free corridor)
        if (r.cost >= 0 && r.path.length > 0 && pathCrosses(r.path, baseNet) && this.useOccupancyToggle) {
          const connMesh = this.buildMeshExcluding(layerZ, c.connNames)
          if (connMesh) r = this.searchPolyanya(connMesh, c.originalStart, c.originalEnd)
        }
        if (r.cost >= 0 && r.path.length > 0 && !pathCrosses(r.path, baseNet)) {
          this.remaining[cand.idx]!.start = c.originalStart
          this.remaining[cand.idx]!.end = c.originalEnd
          return { idx: cand.idx, path: r.path }
        }
      } else if (cand.effectiveS && cand.effectiveE) {
        let r: { cost: number; path: Point[] } = { cost: -1, path: [] }
        if (this.useOccupancyToggle) {
          this.toggleConnectionObstacles(layerZ, c, baseNet, false)
          r = this.searchPolyanya(mesh, cand.effectiveS, cand.effectiveE)
          this.toggleConnectionObstacles(layerZ, c, baseNet, true)
        } else {
          const connMesh = this.buildMeshExcluding(layerZ, c.connNames) ?? mesh
          r = this.searchPolyanya(connMesh, cand.effectiveS, cand.effectiveE)
        }
        if (r.cost >= 0 && r.path.length > 0 && pathCrosses(r.path, baseNet) && this.useOccupancyToggle) {
          const connMesh = this.buildMeshExcluding(layerZ, c.connNames) ?? mesh
          r = this.searchPolyanya(connMesh, cand.effectiveS, cand.effectiveE)
        }
        if (r.cost >= 0 && r.path.length > 0 && !pathCrosses(r.path, baseNet)) {
          this.remaining[cand.idx]!.start = cand.effectiveS
          this.remaining[cand.idx]!.end = cand.effectiveE
          return { idx: cand.idx, path: r.path }
        }
      }
    }

    return { idx: -1, path: [] }
  }

  /**
   * Try to pick a connection across all layers, preferring layer 0.
   * Returns the best match with its layer, or idx=-1 if nothing routable.
   */
  private pickBestAcrossLayers(pickShortest: boolean): {
    idx: number
    path: Point[]
    layerZ: number
  } {
    // Try layer 0 first (primary layer)
    const layer0 = this.pickBestOnLayer(0, pickShortest)
    if (layer0.idx >= 0) {
      return { ...layer0, layerZ: 0 }
    }

    // Fall back to other layers
    for (let z = 1; z < this.layerCount; z++) {
      const result = this.pickBestOnLayer(z, pickShortest)
      if (result.idx >= 0) {
        return { ...result, layerZ: z }
      }
    }

    return { idx: -1, path: [], layerZ: 0 }
  }

  /** Commit a routed path on a specific layer: add to results and update obstacles/regions */
  /**
   * Pathfind a bridge segment from pad center to nudge point on the given
   * layer's mesh. Falls back to a straight line if pathfinding fails (e.g.
   * pad center is inside its own obstacle — expected for short bridges).
   */
  private pathfindBridge(
    from: Point,
    to: Point,
    bridgeLayerZ: number,
  ): Point[] {
    if (Math.abs(from.x - to.x) < 1e-6 && Math.abs(from.y - to.y) < 1e-6)
      return []
    const mesh = this.meshes[bridgeLayerZ]
    if (mesh) {
      const si = new SearchInstance(mesh)
      si.setStartGoal(from, to)
      if (si.search()) {
        const pts = si.getPathPoints()
        if (pts.length >= 2) return pts
      }
    }
    // Fallback: straight line (pad center is typically inside its own obstacle
    // so pathfinding may fail, but the bridge is still valid since it stays
    // within the connected obstacle's footprint)
    return [from, to]
  }

  private commitPath(
    conn: {
      name: string
      originalStart: Point
      originalEnd: Point
      startLayerZ: number
      endLayerZ: number
    },
    path: Point[],
    layerZ: number,
  ) {
    const fullRoute: { x: number; y: number; z: number }[] = []
    const vias: { x: number; y: number }[] = []

    const routeStart = path[0]!
    const routeEnd = path[path.length - 1]!

    const startZ = conn.startLayerZ
    const endZ = conn.endLayerZ
    const needStartVia = startZ !== layerZ
    const needEndVia = endZ !== layerZ

    const startBridged =
      Math.abs(conn.originalStart.x - routeStart.x) > 1e-6 ||
      Math.abs(conn.originalStart.y - routeStart.y) > 1e-6
    const endBridged =
      Math.abs(conn.originalEnd.x - routeEnd.x) > 1e-6 ||
      Math.abs(conn.originalEnd.y - routeEnd.y) > 1e-6

    // Precompute bridge paths (pathfound on native layer mesh)
    const startBridgeZ = needStartVia ? startZ : layerZ
    const endBridgeZ = needEndVia ? endZ : layerZ
    const startBridgePath: Point[] = startBridged
      ? this.pathfindBridge(conn.originalStart, routeStart, startBridgeZ)
      : []
    const endBridgePath: Point[] = endBridged
      ? this.pathfindBridge(routeEnd, conn.originalEnd, endBridgeZ)
      : []

    // === Build fullRoute ===
    // Start bridge on native layer
    if (needStartVia) {
      if (startBridgePath.length > 0) {
        for (const p of startBridgePath) {
          fullRoute.push({ x: p.x, y: p.y, z: startZ })
        }
        // Ensure via point is the last before transition
        const last = fullRoute[fullRoute.length - 1]!
        if (
          Math.abs(last.x - routeStart.x) > 1e-6 ||
          Math.abs(last.y - routeStart.y) > 1e-6
        ) {
          fullRoute.push({ x: routeStart.x, y: routeStart.y, z: startZ })
        }
      } else {
        fullRoute.push({ x: routeStart.x, y: routeStart.y, z: startZ })
      }
      vias.push({ x: routeStart.x, y: routeStart.y })
      fullRoute.push({ x: routeStart.x, y: routeStart.y, z: layerZ })
      for (let i = 1; i < path.length; i++) {
        fullRoute.push({ x: path[i]!.x, y: path[i]!.y, z: layerZ })
      }
    } else {
      // Start bridge on same layer as route
      if (startBridgePath.length > 0) {
        for (let i = 0; i < startBridgePath.length - 1; i++) {
          fullRoute.push({
            x: startBridgePath[i]!.x,
            y: startBridgePath[i]!.y,
            z: layerZ,
          })
        }
      }
      for (const p of path) {
        fullRoute.push({ x: p.x, y: p.y, z: layerZ })
      }
    }

    // End via transition
    if (needEndVia) {
      vias.push({ x: routeEnd.x, y: routeEnd.y })
      fullRoute.push({ x: routeEnd.x, y: routeEnd.y, z: endZ })
    }

    // End bridge on native layer
    if (endBridgePath.length > 0) {
      for (let i = 1; i < endBridgePath.length; i++) {
        fullRoute.push({
          x: endBridgePath[i]!.x,
          y: endBridgePath[i]!.y,
          z: endBridgeZ,
        })
      }
      // Ensure original end is the final point
      const last = fullRoute[fullRoute.length - 1]!
      if (
        Math.abs(last.x - conn.originalEnd.x) > 1e-6 ||
        Math.abs(last.y - conn.originalEnd.y) > 1e-6
      ) {
        fullRoute.push({
          x: conn.originalEnd.x,
          y: conn.originalEnd.y,
          z: endBridgeZ,
        })
      }
    }

    this.resolvedPaths.push({
      connectionName: conn.name,
      route: fullRoute,
      vias,
    })

    // Trace-to-trace clearance: half-width of this trace + half-width of
    // the next trace that needs to pass alongside. Using margin here would
    // be too generous and block adjacent routes unnecessarily.
    const clearance = this.minTraceWidth / 2 + this.margin

    if (this.useObstacles) {
      // Track which layers need mesh rebuild
      const layersToRebuild = new Set<number>()
      layersToRebuild.add(layerZ)

      // --- Main route obstacle on layerZ (includes same-layer bridges) ---
      const routeObstaclePath: Point[] = []
      if (!needStartVia && startBridgePath.length > 0) {
        // Include pathfound bridge in the route obstacle (same layer)
        for (let i = 0; i < startBridgePath.length - 1; i++) {
          routeObstaclePath.push(startBridgePath[i]!)
        }
      }
      for (const p of path) routeObstaclePath.push(p)
      if (!needEndVia && endBridgePath.length > 0) {
        for (let i = 1; i < endBridgePath.length; i++) {
          routeObstaclePath.push(endBridgePath[i]!)
        }
      }

      const newObstacles = this.pathToObstaclePolygons(
        routeObstaclePath,
        clearance,
        conn.originalStart,
        conn.originalEnd,
      )
      for (const poly of newObstacles) {
        this.pushTraceObstacle(layerZ, poly, conn.name)
        this.traceObstaclePolys.push({
          polygon: poly,
          connectionName: conn.name,
          layerZ,
        })
      }

      // --- Bridge obstacles on the endpoint's native layer (when different from route) ---
      const addBridgeObs = (bridgePath: Point[], bridgeLayerZ: number) => {
        if (bridgePath.length < 2) return
        const bridgeObs = this.pathToObstaclePolygons(
          bridgePath,
          clearance,
          conn.originalStart,
          conn.originalEnd,
        )
        for (const poly of bridgeObs) {
          this.pushTraceObstacle(bridgeLayerZ, poly, conn.name)
          this.traceObstaclePolys.push({
            polygon: poly,
            connectionName: conn.name,
            layerZ: bridgeLayerZ,
          })
        }
        layersToRebuild.add(bridgeLayerZ)
      }

      if (needStartVia && startBridgePath.length >= 2) {
        addBridgeObs(startBridgePath, startZ)
      }
      if (needEndVia && endBridgePath.length >= 2) {
        addBridgeObs(endBridgePath, endZ)
      }

      // Add via obstacles to ALL layers (vias are through-hole)
      if (vias.length > 0) {
        for (let z = 0; z < this.layerCount; z++) {
          for (const via of vias) {
            const viaPoly = this.viaToObstaclePolygon(via)
            this.rectObstacles[z]!.push(viaPoly)
          }
          layersToRebuild.add(z)
        }
      }

      // Rebuild meshes for all affected layers
      layersToRebuild.forEach((z) => {
        if (z < this.layerCount) this.buildMesh(z)
      })
    } else {
      const newRegions = this.pathToWeightedRegions(path, clearance)
      this.traceWeightedRegions[layerZ]!.push(...newRegions)

      for (const wr of newRegions) {
        this.traceObstaclePolys.push({
          polygon: wr.polygon,
          connectionName: conn.name,
          layerZ,
        })
      }
    }
  }

  /** Save current results if they're the best so far */
  private saveIfBest() {
    if (
      !this.bestResults ||
      this.resolvedPaths.length > this.bestResults.length
    ) {
      this.bestResults = this.resolvedPaths
      this.bestObstaclePolys = this.traceObstaclePolys
    }
  }

  /** Add a new layer (base obstacle polygons already computed in constructor) */
  private addLayer() {
    this.layerCount++
  }

  /** Advance to the next phase after current phase gets stuck */
  private advancePhase() {
    this.saveIfBest()

    if (this.phase === "shortest") {
      this.resetState()
      this.phase = "longest"
      return
    }

    if (this.phase === "longest") {
      // Try adding a layer if allowed
      if (this.layerCount < this.maxLayerCount) {
        this.addLayer()
        this.resetState()
        this.phase = "shortest"
        return
      }
    }

    // All phases and layers exhausted — use best result
    if (this.bestResults) {
      this.resolvedPaths = this.bestResults
      this.traceObstaclePolys = this.bestObstaclePolys!
    }
    this.phase = "done"
    this.validate()
    this.solved = true
  }

  _step() {
    if (this.phase === "done") {
      this.solved = true
      return
    }

    // Initialize timer on first step
    if (this.solveStartTime === 0) this.solveStartTime = Date.now()

    // Timeout guard
    if (
      Date.now() - this.solveStartTime >
      GreedySequentialPathSolver.MAX_SOLVE_TIME_MS
    ) {
      console.warn(
        `GreedySequentialPathSolver: timeout after ${GreedySequentialPathSolver.MAX_SOLVE_TIME_MS}ms, ${this.resolvedPaths.length}/${this.totalConnections} routed`,
      )
      this.saveIfBest()
      if (this.bestResults) {
        this.resolvedPaths = this.bestResults
        this.traceObstaclePolys = this.bestObstaclePolys!
      }
      this.phase = "done"
      this.validate()
      this.solved = true
      return
    }

    // -----------------------------------------------------------------------
    // Normal routing step (relaxation disabled)
    // -----------------------------------------------------------------------
    if (this.remaining.length === 0) {
      this.saveIfBest()
      this.phase = "done"
      this.validate()
      this.solved = true
      return
    }

    const pickShortest = this.phase === "shortest"
    const { idx, path, layerZ } = this.pickBestAcrossLayers(pickShortest)

    if (idx < 0) {
      // Stuck — advance to next phase
      this.advancePhase()
      return
    }

    const conn = this.remaining[idx]!
    this.remaining.splice(idx, 1)
    this.commitPath(conn, path, layerZ)

    this.progress =
      (this.totalConnections - this.remaining.length) / this.totalConnections
  }

  /** Run post-solve validation: check unrouted connections and same-layer crossings */
  private validate() {
    const unrouted = this.remaining.map((c) => c.name)

    // Build per-connection layer-segmented routes for crossing detection
    type Seg = {
      connName: string
      z: number
      x1: number
      y1: number
      x2: number
      y2: number
    }
    const segments: Seg[] = []
    for (const rp of this.resolvedPaths) {
      for (let i = 0; i < rp.route.length - 1; i++) {
        const a = rp.route[i]!
        const b = rp.route[i + 1]!
        if (a.z !== b.z) continue // skip via transitions
        segments.push({
          connName: rp.connectionName,
          z: a.z,
          x1: a.x,
          y1: a.y,
          x2: b.x,
          y2: b.y,
        })
      }
    }

    // Check all segment pairs from different nets on the same layer
    const crossings: NonNullable<
      typeof this.validationResult
    >["crossNetCrossings"] = []
    // Build a map from connName to root net name for same-net detection
    const connByName = new Map(this.srj.connections.map((c) => [c.name, c]))
    const getNetName = (name: string): string => {
      const conn = connByName.get(name)
      return conn?.rootConnectionName ?? conn?.netConnectionName ?? name
    }

    for (let i = 0; i < segments.length; i++) {
      const a = segments[i]!
      for (let j = i + 1; j < segments.length; j++) {
        const b = segments[j]!
        if (a.z !== b.z) continue
        if (getNetName(a.connName) === getNetName(b.connName)) continue

        const ix = segSegIntersection(
          a.x1,
          a.y1,
          a.x2,
          a.y2,
          b.x1,
          b.y1,
          b.x2,
          b.y2,
        )
        if (ix) {
          crossings.push({
            traceA: a.connName,
            traceB: b.connName,
            layer: a.z,
            point: ix,
          })
        }
      }
    }

    this.validationResult = {
      totalConnections: this.totalConnections,
      routedConnections: this.resolvedPaths.length,
      unroutedConnections: unrouted,
      crossNetCrossings: crossings,
    }
  }

  getResolvedPaths(): ResolvedPath[] {
    return this.resolvedPaths
  }

  /** Connection names that couldn't be routed */
  getUnroutedConnectionNames(): string[] {
    return this.remaining.map((c) => c.name)
  }

  getEffectiveLayerCount(): number {
    return this.layerCount
  }

  /** Convert an hsl/hsla color string to hsla with the given alpha (0-1). */
  private withAlpha(color: string, alpha: number): string {
    const m = color.match(/^hsl\(([^)]+)\)$/)
    if (m) return `hsla(${m[1]}, ${alpha})`
    const ma = color.match(/^hsla\((.+),\s*[\d.]+\)$/)
    if (ma) return `hsla(${ma[1]}, ${alpha})`
    return color
  }

  visualize(): GraphicsObject {
    const lines: Line[] = []
    const points: GraphicsObject["points"] = []

    // Draw committed trace paths, split by layer for distinct styling
    for (const rp of this.resolvedPaths) {
      const color = this.colorMap[rp.connectionName] ?? "green"
      // Group consecutive points by z-layer
      let segStart = 0
      for (let i = 1; i <= rp.route.length; i++) {
        const prevZ = rp.route[i - 1]!.z
        const curZ = i < rp.route.length ? rp.route[i]!.z : -999
        if (curZ !== prevZ || i === rp.route.length) {
          const segment = rp.route.slice(segStart, i)
          if (segment.length >= 2) {
            const isTop = prevZ === 0
            lines.push({
              points: segment.map((p) => ({ x: p.x, y: p.y })),
              strokeColor: isTop ? color : this.withAlpha(color, 0.6),
              strokeWidth: this.minTraceWidth,
              ...(isTop ? {} : { strokeDash: "4 2" }),
            })
          }
          segStart = i
        }
      }
    }

    // Draw capsule collision bounds as one thick polyline per same-layer
    // run.  Drawing the whole run as a single polyline (rather than one
    // line per segment) means joints are filled naturally — no wedge-shaped
    // gaps at bends.  The Minkowski sum of a polyline with a circle is
    // exactly a chain of capsules with filled joints.
    if (this.relaxState !== "idle") {
      const capsuleDiameter = (this.minTraceWidth / 2 + this.margin) * 2
      for (const rp of this.resolvedPaths) {
        const color = this.colorMap[rp.connectionName] ?? "green"
        // Collect same-layer runs
        let runStart = 0
        for (let i = 1; i <= rp.route.length; i++) {
          const prevZ = rp.route[i - 1]!.z
          const curZ = i < rp.route.length ? rp.route[i]!.z : -1
          if (curZ !== prevZ || i === rp.route.length) {
            if (prevZ === 0 && i - runStart >= 2) {
              const pts = rp.route
                .slice(runStart, i)
                .map((p) => ({ x: p.x, y: p.y }))
              lines.push({
                points: pts,
                strokeColor: this.withAlpha(color, 0.18),
                strokeWidth: capsuleDiameter,
              })
            }
            runStart = i
          }
        }
      }
    }

    // Draw convex region map (layer 0): filled polygons colored by state
    const mesh = this.meshes[0]
    if (mesh) {
      const rects: GraphicsObject["rects"] = []
      for (let pi = 0; pi < mesh.polygons.length; pi++) {
        const polygon = mesh.polygons[pi]!
        if (polygon.vertices.length < 3) continue

        // Compute AABB for the rect visualization
        let minPx = Infinity
        let minPy = Infinity
        let maxPx = -Infinity
        let maxPy = -Infinity
        const polyPts: { x: number; y: number }[] = []
        for (const vIdx of polygon.vertices) {
          const v = mesh.vertices[vIdx]
          if (!v) continue
          polyPts.push({ x: v.p.x, y: v.p.y })
          if (v.p.x < minPx) minPx = v.p.x
          if (v.p.y < minPy) minPy = v.p.y
          if (v.p.x > maxPx) maxPx = v.p.x
          if (v.p.y > maxPy) maxPy = v.p.y
        }

        // Draw polygon outline with color based on state
        const isBlocked = polygon.blocked
        const isObstacle = polygon.obstacleIndex >= 0

        // Color: blocked obstacles = red fill, free space = faint blue,
        // unblocked obstacles = green (shouldn't normally appear during viz)
        const fillColor = isBlocked
          ? "rgba(255,60,60,0.15)"
          : isObstacle
            ? "rgba(60,255,60,0.1)"
            : "rgba(80,120,255,0.04)"

        const strokeColor = isBlocked
          ? "rgba(255,80,80,0.5)"
          : isObstacle
            ? "rgba(60,200,60,0.3)"
            : "rgba(100,100,255,0.2)"

        // Draw as closed polyline (more accurate than rect for non-axis-aligned)
        if (polyPts.length >= 3) {
          const closedPts = [...polyPts, { ...polyPts[0]! }]
          lines.push({
            points: closedPts,
            strokeColor,
            strokeWidth: isBlocked ? 0.06 : 0.02,
          })
        }
      }
    }

    // Draw trace obstacle polygon outlines (thicker, on top)
    for (const tp of this.traceObstaclePolys) {
      const color = this.colorMap[tp.connectionName] ?? "green"
      const alpha = tp.layerZ === 0 ? 0.6 : 0.3
      if (tp.polygon.length >= 3) {
        const pts = tp.polygon.map((p) => ({ x: p.x, y: p.y }))
        pts.push({ ...pts[0]! }) // close the polygon
        lines.push({
          points: pts,
          strokeColor: this.withAlpha(color, alpha),
          strokeWidth: 0.05,
        })
      }
    }

    // Draw via markers
    for (const rp of this.resolvedPaths) {
      for (const via of rp.vias) {
        const color = this.colorMap[rp.connectionName] ?? "green"
        points.push({
          x: via.x,
          y: via.y,
          color,
          label: `via`,
        })
      }
    }

    // Mark remaining unrouted endpoints
    for (const conn of this.remaining) {
      const color = this.colorMap[conn.name] ?? "red"
      points.push({
        x: conn.start.x,
        y: conn.start.y,
        color,
        label: `${conn.name} (unrouted)`,
      })
      points.push({ x: conn.end.x, y: conn.end.y, color })
    }

    return { lines, points }
  }
}

/** Strict interior segment-segment intersection (excludes shared endpoints). */
function segSegIntersection(
  ax1: number,
  ay1: number,
  ax2: number,
  ay2: number,
  bx1: number,
  by1: number,
  bx2: number,
  by2: number,
): { x: number; y: number } | null {
  const d1x = ax2 - ax1
  const d1y = ay2 - ay1
  const d2x = bx2 - bx1
  const d2y = by2 - by1
  const denom = d1x * d2y - d1y * d2x
  if (Math.abs(denom) < 1e-12) return null
  const t = ((bx1 - ax1) * d2y - (by1 - ay1) * d2x) / denom
  const u = ((bx1 - ax1) * d1y - (by1 - ay1) * d1x) / denom
  const EPS = 1e-6
  if (t > EPS && t < 1 - EPS && u > EPS && u < 1 - EPS) {
    return { x: ax1 + t * d1x, y: ay1 + t * d1y }
  }
  return null
}
