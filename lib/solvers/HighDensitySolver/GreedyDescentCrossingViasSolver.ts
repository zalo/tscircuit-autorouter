import { BaseSolver } from "lib/solvers/BaseSolver"
import {
  NodeWithPortPoints,
  HighDensityIntraNodeRoute,
} from "lib/types/high-density-types"
import { distance, pointToSegmentDistance } from "@tscircuit/math-utils"
import type { GraphicsObject } from "graphics-debug"
import { safeTransparentize } from "lib/solvers/colors"
import { getBoundsFromNodeWithPortPoints } from "lib/utils/getBoundsFromNodeWithPortPoints"
import { getMinDistBetweenEnteringPoints } from "lib/utils/getMinDistBetweenEnteringPoints"
import { generateColorMapFromNodeWithPortPoints } from "lib/utils/generateColorMapFromNodeWithPortPoints"
import { SingleHighDensityRouteSolver } from "./SingleHighDensityRouteSolver"

type Point3 = { x: number; y: number; z: number }
type Connection = {
  connectionName: string
  A: Point3
  B: Point3
}

/**
 * A greedy routing strategy where each trace descends toward its target using
 * A* pathfinding (via SingleHighDensityRouteSolver) to navigate around all
 * obstacles, and vias are added only at crossing points to resolve same-layer
 * trace conflicts.
 *
 * Algorithm (one connection per _step call):
 * 1. Pick the unrouted connection with fewest same-layer crossings
 * 2. Use the existing A* solver to find a path that avoids all obstacles
 *    (other routes, vias, port points, bounds)
 * 3. Detect same-layer crossings with existing routes along the A* path
 * 4. Insert via hops at crossing regions
 */
export class GreedyDescentCrossingViasSolver extends BaseSolver {
  override getSolverName(): string {
    return "GreedyDescentCrossingViasSolver"
  }

  nodeWithPortPoints: NodeWithPortPoints
  connections: Connection[]
  availableConnections: Connection[]
  solvedRoutes: HighDensityIntraNodeRoute[] = []
  viaDiameter: number
  traceThickness: number
  obstacleMargin: number
  bounds: { minX: number; maxX: number; minY: number; maxY: number }
  colorMap: Record<string, string>
  minDistBetweenEnteringPoints: number
  layerCount: number
  retryCount: Map<string, number>

  constructor(params: {
    nodeWithPortPoints: NodeWithPortPoints
    viaDiameter?: number
    traceThickness?: number
    obstacleMargin?: number
  }) {
    super()
    this.nodeWithPortPoints = params.nodeWithPortPoints
    this.viaDiameter = params.viaDiameter ?? 0.6
    this.traceThickness = params.traceThickness ?? 0.15
    this.obstacleMargin = params.obstacleMargin ?? 0.1
    this.bounds = getBoundsFromNodeWithPortPoints(this.nodeWithPortPoints)
    this.connections = this.extractConnections()
    this.availableConnections = [...this.connections]
    this.colorMap = generateColorMapFromNodeWithPortPoints(
      this.nodeWithPortPoints,
    )
    this.minDistBetweenEnteringPoints = getMinDistBetweenEnteringPoints(
      this.nodeWithPortPoints,
    )
    this.layerCount = this.nodeWithPortPoints.portPoints.reduce(
      (max, p) => Math.max(max, (p.z ?? 0) + 1),
      2,
    )

    this.retryCount = new Map()
    this.MAX_ITERATIONS = this.connections.length * 3 + 5

    if (this.connections.length === 0) {
      this.solved = true
      this.progress = 1
    }
  }

  private extractConnections(): Connection[] {
    const groups = new Map<string, Point3[]>()
    for (const pp of this.nodeWithPortPoints.portPoints) {
      if (!groups.has(pp.connectionName)) groups.set(pp.connectionName, [])
      groups.get(pp.connectionName)!.push({ x: pp.x, y: pp.y, z: pp.z ?? 0 })
    }
    const connections: Connection[] = []
    for (const [connectionName, points] of groups) {
      if (points.length >= 2) {
        connections.push({ connectionName, A: points[0], B: points[1] })
      }
    }
    return connections
  }

  // ─── Segment intersection ────────────────────────────────────────────

  private segmentIntersectionT(
    P1: { x: number; y: number },
    P2: { x: number; y: number },
    Q1: { x: number; y: number },
    Q2: { x: number; y: number },
  ): number | null {
    const dx = P2.x - P1.x
    const dy = P2.y - P1.y
    const ex = Q2.x - Q1.x
    const ey = Q2.y - Q1.y
    const denom = dx * ey - dy * ex
    if (Math.abs(denom) < 1e-10) return null
    const t = ((Q1.x - P1.x) * ey - (Q1.y - P1.y) * ex) / denom
    const u = ((Q1.x - P1.x) * dy - (Q1.y - P1.y) * dx) / denom
    if (t > 0.001 && t < 0.999 && u > 0.001 && u < 0.999) return t
    return null
  }

  // ─── Crossing helpers ────────────────────────────────────────────────

  private countSameLayerCrossings(conn: Connection): number {
    if (conn.A.z !== conn.B.z) return 0
    let count = 0
    const z = conn.A.z
    for (const route of this.solvedRoutes) {
      for (let i = 0; i < route.route.length - 1; i++) {
        const sA = route.route[i]
        const sB = route.route[i + 1]
        if (sA.z !== z || sB.z !== z) continue
        if (this.segmentIntersectionT(conn.A, conn.B, sA, sB) !== null) count++
      }
    }
    return count
  }

  private getAlternateLayer(z: number): number {
    return z === 1 ? 0 : 1
  }

  // ─── Use the repo's existing A* to find a single-connection path ─────

  /**
   * Build obstacle routes representing foreign port points.  Each port is
   * modelled as a **via** so that `isNodeTooCloseToObstacle` uses the
   * reliable circular-distance check (works regardless of cell step).
   * Segment-based obstacles silently fail when the grid is coarse.
   */
  private buildPortPointObstacleRoutes(
    connectionName: string,
    A: Point3,
    B: Point3,
  ): HighDensityIntraNodeRoute[] {
    const obstacles: HighDensityIntraNodeRoute[] = []
    const corridorMargin = Math.max(
      this.viaDiameter * 3,
      distance(A, B) * 0.3,
    )

    for (const pp of this.nodeWithPortPoints.portPoints) {
      if (pp.connectionName === connectionName) continue
      const minX = Math.min(A.x, B.x) - corridorMargin
      const maxX = Math.max(A.x, B.x) + corridorMargin
      const minY = Math.min(A.y, B.y) - corridorMargin
      const maxY = Math.max(A.y, B.y) + corridorMargin
      if (pp.x < minX || pp.x > maxX || pp.y < minY || pp.y > maxY) continue

      const z = pp.z ?? 0
      // Model the port as a via – the A*'s via proximity check is a
      // reliable circular exclusion that works at any cell step.
      obstacles.push({
        connectionName: pp.connectionName,
        route: [
          { x: pp.x, y: pp.y, z },
          { x: pp.x, y: pp.y, z },
        ],
        traceThickness: 0,
        viaDiameter: this.viaDiameter,
        vias: [{ x: pp.x, y: pp.y }],
      })
    }

    return obstacles
  }

  /**
   * Check whether a solved route passes too close to any port that does not
   * belong to this connection.
   */
  private pathHitsForeignPort(
    route: HighDensityIntraNodeRoute,
    connectionName: string,
  ): boolean {
    const threshold = this.viaDiameter / 2 + this.traceThickness / 2
    for (const pp of this.nodeWithPortPoints.portPoints) {
      if (pp.connectionName === connectionName) continue
      const z = pp.z ?? 0
      for (let i = 0; i < route.route.length - 1; i++) {
        if (route.route[i].z !== z || route.route[i + 1].z !== z) continue
        const dist = pointToSegmentDistance(
          { x: pp.x, y: pp.y },
          route.route[i],
          route.route[i + 1],
        )
        if (dist < threshold) return true
      }
    }
    return false
  }

  private solveSubpath(
    A: Point3,
    B: Point3,
    connectionName: string,
    obstacleRoutes: HighDensityIntraNodeRoute[],
  ): HighDensityIntraNodeRoute | null {
    const subSolver = new SingleHighDensityRouteSolver({
      connectionName,
      A,
      B,
      bounds: this.bounds,
      obstacleRoutes,
      minDistBetweenEnteringPoints: this.minDistBetweenEnteringPoints,
      viaDiameter: this.viaDiameter,
      traceThickness: this.traceThickness,
      layerCount: this.layerCount,
      futureConnections: [],
      hyperParameters: {
        CELL_SIZE_FACTOR: 0.5, // finer grid for port avoidance
      },
    })

    subSolver.MAX_ITERATIONS = 8000
    subSolver.solve()
    return subSolver.solvedPath
  }

  // ─── Crossing detection along a multi-segment path ───────────────────

  private findCrossingDistances(pathPoints: Point3[], z: number): number[] {
    const cumDist = cumulativeDistances(pathPoints)
    const crossings: number[] = []

    for (let i = 0; i < pathPoints.length - 1; i++) {
      const pA = pathPoints[i]
      const pB = pathPoints[i + 1]
      if (pA.z !== z || pB.z !== z) continue

      for (const route of this.solvedRoutes) {
        for (let j = 0; j < route.route.length - 1; j++) {
          const eA = route.route[j]
          const eB = route.route[j + 1]
          if (eA.z !== z || eB.z !== z) continue
          const t = this.segmentIntersectionT(pA, pB, eA, eB)
          if (t !== null) {
            crossings.push(cumDist[i] + t * (cumDist[i + 1] - cumDist[i]))
          }
        }
      }
    }

    crossings.sort((a, b) => a - b)
    return crossings
  }

  // ─── Via-hop insertion along a linearised path ───────────────────────

  private addViaHops(
    pathPoints: Point3[],
    crossingDists: number[],
    z: number,
  ): { route: Point3[]; vias: { x: number; y: number }[] } {
    const cumDist = cumulativeDistances(pathPoints)
    const totalDist = cumDist[cumDist.length - 1]

    const altLayer = this.getAlternateLayer(z)
    const viaOffset = this.viaDiameter + this.obstacleMargin * 2

    // Merge crossings into hop regions
    const regions: { start: number; end: number }[] = []
    for (const d of crossingDists) {
      const s = Math.max(viaOffset, d - viaOffset)
      const e = Math.min(totalDist - viaOffset, d + viaOffset)
      if (regions.length > 0 && regions[regions.length - 1].end >= s - 0.01) {
        regions[regions.length - 1].end = Math.max(
          regions[regions.length - 1].end,
          e,
        )
      } else {
        regions.push({ start: s, end: e })
      }
    }

    // Interpolate a point at distance d along the path
    const pointAt = (d: number): { x: number; y: number } => {
      d = Math.max(0, Math.min(totalDist, d))
      for (let i = 0; i < cumDist.length - 1; i++) {
        if (d <= cumDist[i + 1] + 1e-6) {
          const segLen = cumDist[i + 1] - cumDist[i]
          if (segLen < 1e-10) continue
          const t = clamp01((d - cumDist[i]) / segLen)
          return {
            x: pathPoints[i].x + t * (pathPoints[i + 1].x - pathPoints[i].x),
            y: pathPoints[i].y + t * (pathPoints[i + 1].y - pathPoints[i].y),
          }
        }
      }
      const last = pathPoints[pathPoints.length - 1]
      return { x: last.x, y: last.y }
    }

    // Build event list: waypoints + via transitions, sorted by distance
    type Evt =
      | { dist: number; kind: "wp"; p: Point3 }
      | { dist: number; kind: "via_down" }
      | { dist: number; kind: "via_up" }

    const events: Evt[] = []
    for (let i = 0; i < pathPoints.length; i++) {
      events.push({ dist: cumDist[i], kind: "wp", p: pathPoints[i] })
    }
    for (const r of regions) {
      events.push({ dist: r.start, kind: "via_down" })
      events.push({ dist: r.end, kind: "via_up" })
    }
    events.sort((a, b) => a.dist - b.dist)

    const route: Point3[] = []
    const vias: { x: number; y: number }[] = []
    let curZ = z

    for (const ev of events) {
      const p = ev.kind === "wp" ? ev.p : pointAt(ev.dist)
      if (ev.kind === "wp") {
        route.push({ x: p.x, y: p.y, z: curZ })
      } else if (ev.kind === "via_down") {
        route.push({ x: p.x, y: p.y, z: curZ })
        curZ = altLayer
        route.push({ x: p.x, y: p.y, z: curZ })
        vias.push({ x: p.x, y: p.y })
      } else {
        route.push({ x: p.x, y: p.y, z: curZ })
        curZ = z
        route.push({ x: p.x, y: p.y, z: curZ })
        vias.push({ x: p.x, y: p.y })
      }
    }

    // Safety: if still on alt layer at end, via back
    if (curZ !== z && route.length > 0) {
      const last = route[route.length - 1]
      route.push({ x: last.x, y: last.y, z })
      vias.push({ x: last.x, y: last.y })
    }

    return { route: deduplicateRoute(route), vias }
  }

  // ─── Multi-step _step: route one connection per call ─────────────────

  _step() {
    if (this.availableConnections.length === 0) {
      this.solved = true
      this.progress = 1
      return
    }

    // Pick the connection with fewest crossings against already-solved routes
    let bestIdx = 0
    let bestScore = Infinity
    for (let i = 0; i < this.availableConnections.length; i++) {
      const score = this.countSameLayerCrossings(this.availableConnections[i])
      if (score < bestScore) {
        bestScore = score
        bestIdx = i
      }
      if (score === 0) break
    }

    const conn = this.availableConnections.splice(bestIdx, 1)[0]
    const prevRouteCount = this.solvedRoutes.length
    this.routeConnection(conn)

    // Check if the newly-solved route passes through a foreign port.
    // If so, remove it and push the connection to the back of the queue
    // so it is re-attempted after more routes are in place as obstacles.
    const MAX_RETRIES = 3
    if (this.solvedRoutes.length > prevRouteCount) {
      const lastRoute = this.solvedRoutes[this.solvedRoutes.length - 1]
      const retries = this.retryCount.get(conn.connectionName) ?? 0
      if (
        this.pathHitsForeignPort(lastRoute, conn.connectionName) &&
        retries < MAX_RETRIES
      ) {
        this.solvedRoutes.pop()
        this.retryCount.set(conn.connectionName, retries + 1)
        this.availableConnections.push(conn)
        return
      }
    }

    this.progress = this.solvedRoutes.length / this.connections.length
    if (this.availableConnections.length === 0) {
      this.solved = true
      this.progress = 1
    }
  }

  // ─── Route dispatching ───────────────────────────────────────────────

  private routeConnection(conn: Connection) {
    const { A, B, connectionName } = conn
    const sameXY = Math.abs(A.x - B.x) < 1e-6 && Math.abs(A.y - B.y) < 1e-6

    if (sameXY && A.z === B.z) return
    if (sameXY) {
      this.routeSamePointTransition(conn)
      return
    }

    // Solved routes + port-point via obstacles.  The A* handles both
    // obstacle avoidance and layer-change vias natively, so traces
    // will use vias to cross over existing routes rather than running
    // through them.
    const obstacleRoutes = [
      ...this.solvedRoutes,
      ...this.buildPortPointObstacleRoutes(connectionName, A, B),
    ]

    if (A.z !== B.z) {
      this.routeLayerTransition(conn, obstacleRoutes)
      return
    }

    this.routeSameLayerGreedy(conn, obstacleRoutes)
  }

  private routeSamePointTransition(conn: Connection) {
    const { A, B, connectionName } = conn
    const margin = this.viaDiameter / 2 + this.obstacleMargin
    const cx = clampVal(
      this.nodeWithPortPoints.center.x,
      this.bounds.minX + margin,
      this.bounds.maxX - margin,
    )
    const cy = clampVal(
      this.nodeWithPortPoints.center.y,
      this.bounds.minY + margin,
      this.bounds.maxY - margin,
    )
    this.solvedRoutes.push({
      connectionName,
      route: deduplicateRoute([
        { ...A },
        { x: cx, y: cy, z: A.z },
        { x: cx, y: cy, z: B.z },
        { ...B },
      ]),
      traceThickness: this.traceThickness,
      viaDiameter: this.viaDiameter,
      vias: [{ x: cx, y: cy }],
    })
  }

  private routeLayerTransition(
    conn: Connection,
    obstacleRoutes: HighDensityIntraNodeRoute[],
  ) {
    const { A, B, connectionName } = conn

    // Try using the full A* which can handle layer transitions natively
    const solved = this.solveSubpath(A, B, connectionName, obstacleRoutes)
    if (solved) {
      this.solvedRoutes.push(solved)
      return
    }

    // Fallback: simple via at midpoint
    const margin = this.viaDiameter / 2 + this.obstacleMargin
    const viaX = clampVal(
      (A.x + B.x) / 2,
      this.bounds.minX + margin,
      this.bounds.maxX - margin,
    )
    const viaY = clampVal(
      (A.y + B.y) / 2,
      this.bounds.minY + margin,
      this.bounds.maxY - margin,
    )
    this.solvedRoutes.push({
      connectionName,
      route: deduplicateRoute([
        { ...A },
        { x: viaX, y: viaY, z: A.z },
        { x: viaX, y: viaY, z: B.z },
        { ...B },
      ]),
      traceThickness: this.traceThickness,
      viaDiameter: this.viaDiameter,
      vias: [{ x: viaX, y: viaY }],
    })
  }

  private routeSameLayerGreedy(
    conn: Connection,
    obstacleRoutes: HighDensityIntraNodeRoute[],
  ) {
    const { A, B, connectionName } = conn

    // The A* handles obstacle avoidance AND via layer-changes natively.
    // It will greedily descend toward B, using vias to cross over any
    // solved routes that block the path on the same layer.
    const solved = this.solveSubpath(A, B, connectionName, obstacleRoutes)

    if (solved) {
      this.solvedRoutes.push(solved)
      return
    }

    // A* failed – fall back to a straight line (will likely hit a port,
    // get caught by pathHitsForeignPort, and be retried later).
    this.solvedRoutes.push({
      connectionName,
      route: [{ ...A }, { ...B }],
      traceThickness: this.traceThickness,
      viaDiameter: this.viaDiameter,
      vias: [],
    })
  }

  // ─── Visualisation (accumulated steps) ───────────────────────────────

  visualize(): GraphicsObject {
    const graphics: GraphicsObject = {
      lines: [],
      points: [],
      rects: [],
      circles: [],
    }

    // Bounds
    graphics.lines!.push({
      points: [
        { x: this.bounds.minX, y: this.bounds.minY },
        { x: this.bounds.maxX, y: this.bounds.minY },
        { x: this.bounds.maxX, y: this.bounds.maxY },
        { x: this.bounds.minX, y: this.bounds.maxY },
        { x: this.bounds.minX, y: this.bounds.minY },
      ],
      strokeColor: "rgba(255, 0, 0, 0.25)",
      strokeDash: "4 4",
      layer: "border",
    })

    // Port points
    for (const pt of this.nodeWithPortPoints.portPoints) {
      graphics.points!.push({
        x: pt.x,
        y: pt.y,
        label: [pt.connectionName, `layer: ${pt.z}`].join("\n"),
        color: this.colorMap[pt.connectionName] ?? "blue",
      })
    }

    // Draw each solved route with its step index
    for (let ri = 0; ri < this.solvedRoutes.length; ri++) {
      const route = this.solvedRoutes[ri]
      const routeColor = this.colorMap[route.connectionName] ?? "blue"

      for (let i = 0; i < route.route.length - 1; i++) {
        const p1 = route.route[i]
        const p2 = route.route[i + 1]
        graphics.lines!.push({
          points: [p1, p2],
          strokeColor:
            p1.z === p2.z && p1.z === 0
              ? safeTransparentize(routeColor, 0.2)
              : safeTransparentize(routeColor, 0.6),
          layer: `route-layer-${p1.z}`,
          step: ri,
          strokeWidth: route.traceThickness,
        })
      }

      for (const via of route.vias) {
        graphics.circles!.push({
          center: { x: via.x, y: via.y },
          radius: route.viaDiameter / 2,
          fill: safeTransparentize(routeColor, 0.5),
          layer: "via",
          step: ri,
        })
      }
    }

    return graphics
  }
}

// ─── Utilities ──────────────────────────────────────────────────────────

function deduplicateRoute(route: Point3[]): Point3[] {
  if (route.length === 0) return route
  const result: Point3[] = [route[0]]
  for (let i = 1; i < route.length; i++) {
    const prev = result[result.length - 1]
    const curr = route[i]
    if (
      Math.abs(prev.x - curr.x) > 1e-6 ||
      Math.abs(prev.y - curr.y) > 1e-6 ||
      prev.z !== curr.z
    ) {
      result.push(curr)
    }
  }
  return result
}

function cumulativeDistances(pts: { x: number; y: number }[]): number[] {
  const cd = [0]
  for (let i = 1; i < pts.length; i++) {
    cd.push(cd[i - 1] + distance(pts[i - 1], pts[i]))
  }
  return cd
}

function clampVal(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v))
}

function clamp01(v: number): number {
  return v < 0 ? 0 : v > 1 ? 1 : v
}
