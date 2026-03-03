import type { GraphicsObject } from "graphics-debug"
import { getSegmentIntersection } from "@tscircuit/math-utils"
import {
  type Mesh,
  SearchInstance,
  distance,
  PointLocationType,
  type Point,
  cdtTriangulate,
  rectToPolygon,
  buildMeshFromRegions,
  mergeMesh,
} from "polyanya"
import { mergeOverlappingRects } from "./mergeOverlappingRects"
import { BaseSolver } from "../BaseSolver"
import type { SimpleRouteJson } from "../../types"
import type { PolyanyaPathResult, ResolvedPath } from "./types"

// ---------- file-local types ----------

interface PathState {
  connectionName: string
  originalStart: Point
  originalEnd: Point
  /** Shared crossing waypoints this path must route through */
  crossingWaypoints: CrossingPoint[]
  currentPath: Point[]
}

/** A single crossing point shared by two paths. Both paths route through it. */
interface CrossingPoint {
  id: number
  pathIndexA: number
  pathIndexB: number
  position: Point
  /** parametricT on path A (for ordering waypoints along a path) */
  parametricTA: number
  /** parametricT on path B */
  parametricTB: number
}

// ---------- constants ----------

const STEP_SIZE = 0.6
const STALL_EPSILON = 1e-4
const MAX_STALL = 10
const MAX_ITERATIONS = 500
const SNAP_ITERATIONS = 16

// ---------- helpers ----------

function snapToMesh(validPt: Point, candidate: Point, mesh: Mesh): Point {
  const loc = mesh.getPointLocation(candidate)
  if (loc.type !== PointLocationType.NOT_ON_MESH) return candidate

  let lo = 0
  let hi = 1
  let best = validPt
  for (let i = 0; i < SNAP_ITERATIONS; i++) {
    const mid = (lo + hi) / 2
    const p: Point = {
      x: validPt.x + (candidate.x - validPt.x) * mid,
      y: validPt.y + (candidate.y - validPt.y) * mid,
    }
    const pLoc = mesh.getPointLocation(p)
    if (pLoc.type !== PointLocationType.NOT_ON_MESH) {
      best = p
      lo = mid
    } else {
      hi = mid
    }
  }
  return best
}

function computeParametricT(
  path: Point[],
  crossingPoint: Point,
  segIndex: number,
): number {
  let cumDist = 0
  for (let i = 0; i < segIndex && i < path.length - 1; i++) {
    cumDist += distance(path[i]!, path[i + 1]!)
  }
  if (segIndex < path.length - 1) {
    cumDist += distance(path[segIndex]!, crossingPoint)
  }
  let totalDist = 0
  for (let i = 0; i < path.length - 1; i++) {
    totalDist += distance(path[i]!, path[i + 1]!)
  }
  return totalDist > 0 ? cumDist / totalDist : 0
}

function repathWithCrossingWaypoints(
  pathState: PathState,
  pathIndex: number,
  searchInstance: SearchInstance,
): Point[] {
  // Get crossing waypoints for this path, sorted by parametricT
  const sorted = [...pathState.crossingWaypoints].sort((a, b) => {
    const tA = a.pathIndexA === pathIndex ? a.parametricTA : a.parametricTB
    const tB = b.pathIndexA === pathIndex ? b.parametricTA : b.parametricTB
    return tA - tB
  })

  // Chain searches: start -> cp1 -> cp2 -> ... -> end
  const waypoints: Point[] = [
    pathState.originalStart,
    ...sorted.map((cp) => cp.position),
    pathState.originalEnd,
  ]

  const segments: Point[][] = []
  for (let i = 0; i < waypoints.length - 1; i++) {
    const from = waypoints[i]!
    const to = waypoints[i + 1]!
    searchInstance.setStartGoal(from, to)
    const found = searchInstance.search()
    if (found) {
      const pts = searchInstance.getPathPoints()
      if (i > 0 && pts.length > 0) {
        segments.push(pts.slice(1))
      } else {
        segments.push(pts)
      }
    } else {
      if (i > 0) {
        segments.push([to])
      } else {
        segments.push([from, to])
      }
    }
  }

  return segments.flat()
}

function detectRawCrossings(
  pathStates: PathState[],
): Array<{
  pathIndexA: number
  pathIndexB: number
  segIndexA: number
  segIndexB: number
  point: Point
}> {
  const crossings: Array<{
    pathIndexA: number
    pathIndexB: number
    segIndexA: number
    segIndexB: number
    point: Point
  }> = []

  for (let i = 0; i < pathStates.length; i++) {
    const pathA = pathStates[i]!.currentPath
    for (let j = i + 1; j < pathStates.length; j++) {
      const pathB = pathStates[j]!.currentPath
      for (let si = 0; si < pathA.length - 1; si++) {
        const a1 = pathA[si]!
        const a2 = pathA[si + 1]!
        for (let sj = 0; sj < pathB.length - 1; sj++) {
          const b1 = pathB[sj]!
          const b2 = pathB[sj + 1]!
          const intersection = getSegmentIntersection(a1, a2, b1, b2)
          if (intersection) {
            crossings.push({
              pathIndexA: i,
              pathIndexB: j,
              segIndexA: si,
              segIndexB: sj,
              point: { x: intersection.x, y: intersection.y },
            })
          }
        }
      }
    }
  }
  return crossings
}

// ---------- solver ----------

export class CrossingRepulsionSolver extends BaseSolver {
  paths: PolyanyaPathResult[]
  mesh: Mesh
  srj: SimpleRouteJson
  colorMap: Record<string, string>
  minTraceWidth: number
  layerCount: number
  viaDiameter: number

  private minCrossingSpacing: number
  private repulsionStrength: number

  private pathStates: PathState[] = []
  private crossingPoints: CrossingPoint[] = []
  private resolvedPaths: ResolvedPath[] = []

  private searchInstance: SearchInstance | null = null
  private phase: "INIT" | "REPULSE" | "FINALIZE" = "INIT"
  private nextCrossingId = 0
  private stallCount = 0
  private prevTotalMovement = 0
  private repulseIterations = 0

  constructor(params: {
    paths: PolyanyaPathResult[]
    mesh: Mesh
    srj: SimpleRouteJson
    colorMap: Record<string, string>
    minTraceWidth: number
    layerCount: number
    viaDiameter: number
  }) {
    super()
    this.paths = params.paths
    this.mesh = params.mesh
    this.srj = params.srj
    this.colorMap = params.colorMap
    this.minTraceWidth = params.minTraceWidth
    this.layerCount = params.layerCount
    this.viaDiameter = params.viaDiameter
    this.MAX_ITERATIONS = MAX_ITERATIONS + 10

    this.minCrossingSpacing = params.viaDiameter * 3
    this.repulsionStrength = params.viaDiameter * 1.5
  }

  _step() {
    switch (this.phase) {
      case "INIT":
        this.initPhase()
        break
      case "REPULSE":
        this.repulseStep()
        break
      case "FINALIZE":
        this.finalizePhase()
        break
    }
  }

  private initPhase() {
    this.searchInstance = new SearchInstance(this.mesh)

    this.pathStates = this.paths.map((p) => ({
      connectionName: p.connectionName,
      originalStart: p.path[0]!,
      originalEnd: p.path[p.path.length - 1]!,
      crossingWaypoints: [],
      currentPath: [...p.path],
    }))

    const rawCrossings = detectRawCrossings(this.pathStates)

    if (rawCrossings.length === 0) {
      this.phase = "FINALIZE"
      return
    }

    // Create one shared CrossingPoint per intersection.
    // Both paths get the same crossing point added as a waypoint.
    for (const rc of rawCrossings) {
      const pathA = this.pathStates[rc.pathIndexA]!
      const pathB = this.pathStates[rc.pathIndexB]!

      const tA = computeParametricT(pathA.currentPath, rc.point, rc.segIndexA)
      const tB = computeParametricT(pathB.currentPath, rc.point, rc.segIndexB)

      const cp: CrossingPoint = {
        id: this.nextCrossingId++,
        pathIndexA: rc.pathIndexA,
        pathIndexB: rc.pathIndexB,
        position: { x: rc.point.x, y: rc.point.y },
        parametricTA: tA,
        parametricTB: tB,
      }

      this.crossingPoints.push(cp)
      pathA.crossingWaypoints.push(cp)
      pathB.crossingWaypoints.push(cp)
    }

    if (this.crossingPoints.length < 2) {
      // Only one crossing — nothing to repel, go straight to via insertion
      this.phase = "FINALIZE"
      return
    }

    this.phase = "REPULSE"
  }

  private repulseStep() {
    this.repulseIterations++

    // 1. Repel crossing points from each other so vias have room
    const forces = new Map<number, Point>()
    for (const cp of this.crossingPoints) {
      forces.set(cp.id, { x: 0, y: 0 })
    }

    for (let i = 0; i < this.crossingPoints.length; i++) {
      const cpA = this.crossingPoints[i]!
      for (let j = i + 1; j < this.crossingPoints.length; j++) {
        const cpB = this.crossingPoints[j]!
        const dx = cpA.position.x - cpB.position.x
        const dy = cpA.position.y - cpB.position.y
        const dist = Math.hypot(dx, dy)

        if (dist < this.minCrossingSpacing) {
          let fx: number
          let fy: number

          if (dist < 1e-6) {
            const angle = Math.random() * Math.PI * 2
            fx = Math.cos(angle) * this.repulsionStrength
            fy = Math.sin(angle) * this.repulsionStrength
          } else {
            const strength =
              this.repulsionStrength *
              ((this.minCrossingSpacing - dist) / this.minCrossingSpacing)
            const ux = dx / dist
            const uy = dy / dist
            fx = ux * strength
            fy = uy * strength
          }

          const fA = forces.get(cpA.id)!
          fA.x += fx
          fA.y += fy
          const fB = forces.get(cpB.id)!
          fB.x -= fx
          fB.y -= fy
        }
      }
    }

    // 2. String-pulling: attract each crossing point toward the straight line
    //    between its neighbors on each path, keeping paths taut
    const STRING_PULL_STRENGTH = this.repulsionStrength * 0.5
    for (const cp of this.crossingPoints) {
      const f = forces.get(cp.id)!

      // For each of the two paths this crossing belongs to, find prev/next anchors
      for (const pathIdx of [cp.pathIndexA, cp.pathIndexB]) {
        const ps = this.pathStates[pathIdx]!
        const isA = pathIdx === cp.pathIndexA
        const myT = isA ? cp.parametricTA : cp.parametricTB

        // Sort this path's crossing waypoints by parametricT
        const sorted = [...ps.crossingWaypoints].sort((a, b) => {
          const tA =
            a.pathIndexA === pathIdx ? a.parametricTA : a.parametricTB
          const tB =
            b.pathIndexA === pathIdx ? b.parametricTA : b.parametricTB
          return tA - tB
        })

        const myIndex = sorted.findIndex((c) => c.id === cp.id)
        const prev: Point =
          myIndex > 0 ? sorted[myIndex - 1]!.position : ps.originalStart
        const next: Point =
          myIndex < sorted.length - 1
            ? sorted[myIndex + 1]!.position
            : ps.originalEnd

        // Project crossing point onto the line prev→next
        const lx = next.x - prev.x
        const ly = next.y - prev.y
        const lenSq = lx * lx + ly * ly
        if (lenSq < 1e-10) continue

        const t = Math.max(
          0,
          Math.min(
            1,
            ((cp.position.x - prev.x) * lx + (cp.position.y - prev.y) * ly) /
              lenSq,
          ),
        )
        const projX = prev.x + lx * t
        const projY = prev.y + ly * t

        // Pull toward projected point
        const pullDx = projX - cp.position.x
        const pullDy = projY - cp.position.y
        const pullDist = Math.hypot(pullDx, pullDy)
        if (pullDist > 1e-6) {
          f.x += (pullDx / pullDist) * Math.min(pullDist, STRING_PULL_STRENGTH)
          f.y += (pullDy / pullDist) * Math.min(pullDist, STRING_PULL_STRENGTH)
        }
      }
    }

    // 3. Apply combined forces (repulsion + string pull), snap to mesh
    let totalMovement = 0
    const dirtyPaths = new Set<number>()

    for (const cp of this.crossingPoints) {
      const f = forces.get(cp.id)!
      if (Math.abs(f.x) < 1e-10 && Math.abs(f.y) < 1e-10) continue

      const candidate: Point = {
        x: cp.position.x + f.x * STEP_SIZE,
        y: cp.position.y + f.y * STEP_SIZE,
      }
      const snapped = snapToMesh(cp.position, candidate, this.mesh)

      totalMovement += distance(cp.position, snapped)
      cp.position = snapped
      dirtyPaths.add(cp.pathIndexA)
      dirtyPaths.add(cp.pathIndexB)
    }

    // 4. Re-pathfind dirty paths through their updated crossing waypoints
    for (const pathIdx of dirtyPaths) {
      const ps = this.pathStates[pathIdx]!
      ps.currentPath = repathWithCrossingWaypoints(
        ps,
        pathIdx,
        this.searchInstance!,
      )
    }

    // 5. Prune stale crossings and detect new ones
    this.pruneAndDetectCrossings(dirtyPaths)

    // 6. Check convergence: all pairs separated by >= minCrossingSpacing
    let allSeparated = true
    for (let i = 0; i < this.crossingPoints.length; i++) {
      for (let j = i + 1; j < this.crossingPoints.length; j++) {
        const d = distance(
          this.crossingPoints[i]!.position,
          this.crossingPoints[j]!.position,
        )
        if (d < this.minCrossingSpacing) {
          allSeparated = false
          break
        }
      }
      if (!allSeparated) break
    }

    if (allSeparated) {
      this.phase = "FINALIZE"
      return
    }

    // Stall detection
    if (Math.abs(totalMovement - this.prevTotalMovement) < STALL_EPSILON) {
      this.stallCount++
    } else {
      this.stallCount = 0
    }
    this.prevTotalMovement = totalMovement

    if (
      this.stallCount >= MAX_STALL ||
      this.repulseIterations >= MAX_ITERATIONS
    ) {
      this.phase = "FINALIZE"
    }

    this.progress = this.repulseIterations / MAX_ITERATIONS
  }

  private pruneAndDetectCrossings(dirtyPaths: Set<number>) {
    const pruneRadius = this.minCrossingSpacing * 2
    const staleIds = new Set<number>()

    // Check each crossing: do the two paths still actually intersect near it?
    for (const cp of this.crossingPoints) {
      const pathA = this.pathStates[cp.pathIndexA]!.currentPath
      const pathB = this.pathStates[cp.pathIndexB]!.currentPath

      let hasNearbyIntersection = false
      for (let si = 0; si < pathA.length - 1 && !hasNearbyIntersection; si++) {
        const a1 = pathA[si]!
        const a2 = pathA[si + 1]!
        const segMidAx = (a1.x + a2.x) / 2
        const segMidAy = (a1.y + a2.y) / 2
        if (
          Math.hypot(segMidAx - cp.position.x, segMidAy - cp.position.y) >
          pruneRadius
        )
          continue

        for (let sj = 0; sj < pathB.length - 1; sj++) {
          const b1 = pathB[sj]!
          const b2 = pathB[sj + 1]!
          const segMidBx = (b1.x + b2.x) / 2
          const segMidBy = (b1.y + b2.y) / 2
          if (
            Math.hypot(segMidBx - cp.position.x, segMidBy - cp.position.y) >
            pruneRadius
          )
            continue

          if (getSegmentIntersection(a1, a2, b1, b2)) {
            hasNearbyIntersection = true
            break
          }
        }
      }

      if (!hasNearbyIntersection) {
        staleIds.add(cp.id)
      }
    }

    // Prune back-to-back crossings for the same pair of traces.
    // If two crossings share the same (pathA, pathB) pair AND are adjacent on
    // both paths (no other crossing between them on either path), then both
    // paths are colinear between those two waypoints. Keep only the one closer
    // to the centroid of the pair.
    for (const ps of this.pathStates) {
      const pathIdx = this.pathStates.indexOf(ps)
      // Get this path's crossings sorted by parametricT (excluding already stale)
      const sorted = ps.crossingWaypoints
        .filter((cp) => !staleIds.has(cp.id))
        .sort((a, b) => {
          const tA =
            a.pathIndexA === pathIdx ? a.parametricTA : a.parametricTB
          const tB =
            b.pathIndexA === pathIdx ? b.parametricTA : b.parametricTB
          return tA - tB
        })

      for (let i = 0; i < sorted.length - 1; i++) {
        const cpA = sorted[i]!
        const cpB = sorted[i + 1]!
        if (staleIds.has(cpA.id) || staleIds.has(cpB.id)) continue

        // Check if they share the same pair of paths
        const pairA = `${Math.min(cpA.pathIndexA, cpA.pathIndexB)}:${Math.max(cpA.pathIndexA, cpA.pathIndexB)}`
        const pairB = `${Math.min(cpB.pathIndexA, cpB.pathIndexB)}:${Math.max(cpB.pathIndexA, cpB.pathIndexB)}`
        if (pairA !== pairB) continue

        // They're adjacent on this path and share the same trace pair.
        // Check they're also adjacent on the other path.
        const otherPathIdx =
          cpA.pathIndexA === pathIdx ? cpA.pathIndexB : cpA.pathIndexA
        const otherPs = this.pathStates[otherPathIdx]!
        const otherSorted = otherPs.crossingWaypoints
          .filter((cp) => !staleIds.has(cp.id))
          .sort((a, b) => {
            const tA =
              a.pathIndexA === otherPathIdx ? a.parametricTA : a.parametricTB
            const tB =
              b.pathIndexA === otherPathIdx ? b.parametricTA : b.parametricTB
            return tA - tB
          })
        const idxA = otherSorted.findIndex((cp) => cp.id === cpA.id)
        const idxB = otherSorted.findIndex((cp) => cp.id === cpB.id)
        if (Math.abs(idxA - idxB) !== 1) continue

        // Back-to-back on both paths — keep the one closer to centroid
        const cx = (cpA.position.x + cpB.position.x) / 2
        const cy = (cpA.position.y + cpB.position.y) / 2
        const dA = Math.hypot(cpA.position.x - cx, cpA.position.y - cy)
        const dB = Math.hypot(cpB.position.x - cx, cpB.position.y - cy)
        staleIds.add(dA <= dB ? cpB.id : cpA.id)
      }
    }

    // Bypass pruning: for each crossing, check if both paths can skip the
    // waypoint entirely (pathfind prev→next) without crossing the other path.
    // If so, the crossing is no longer needed.
    for (const cp of this.crossingPoints) {
      if (staleIds.has(cp.id)) continue

      let canBypassBoth = true
      for (const pathIdx of [cp.pathIndexA, cp.pathIndexB]) {
        const otherPathIdx =
          pathIdx === cp.pathIndexA ? cp.pathIndexB : cp.pathIndexA
        const ps = this.pathStates[pathIdx]!
        const otherPath = this.pathStates[otherPathIdx]!.currentPath

        // Get sorted waypoints for this path, excluding stale ones and this cp
        const sorted = ps.crossingWaypoints
          .filter((c) => !staleIds.has(c.id) && c.id !== cp.id)
          .sort((a, b) => {
            const tA =
              a.pathIndexA === pathIdx ? a.parametricTA : a.parametricTB
            const tB =
              b.pathIndexA === pathIdx ? b.parametricTA : b.parametricTB
            return tA - tB
          })

        const myT =
          cp.pathIndexA === pathIdx ? cp.parametricTA : cp.parametricTB

        // Find prev and next anchors
        let prevPt: Point = ps.originalStart
        let nextPt: Point = ps.originalEnd
        for (let k = 0; k < sorted.length; k++) {
          const t =
            sorted[k]!.pathIndexA === pathIdx
              ? sorted[k]!.parametricTA
              : sorted[k]!.parametricTB
          if (t < myT) {
            prevPt = sorted[k]!.position
          } else {
            nextPt = sorted[k]!.position
            break
          }
        }

        // Pathfind prev → next skipping this waypoint
        this.searchInstance!.setStartGoal(prevPt, nextPt)
        const found = this.searchInstance!.search()
        if (!found) {
          canBypassBoth = false
          break
        }
        const bypassPath = this.searchInstance!.getPathPoints()

        // Check if the bypass path crosses the other path
        let crossesOther = false
        for (
          let si = 0;
          si < bypassPath.length - 1 && !crossesOther;
          si++
        ) {
          const a1 = bypassPath[si]!
          const a2 = bypassPath[si + 1]!
          for (let sj = 0; sj < otherPath.length - 1; sj++) {
            const b1 = otherPath[sj]!
            const b2 = otherPath[sj + 1]!
            if (getSegmentIntersection(a1, a2, b1, b2)) {
              crossesOther = true
              break
            }
          }
        }

        if (crossesOther) {
          canBypassBoth = false
          break
        }
      }

      if (canBypassBoth) {
        staleIds.add(cp.id)
      }
    }

    if (staleIds.size > 0) {
      this.crossingPoints = this.crossingPoints.filter(
        (cp) => !staleIds.has(cp.id),
      )
      for (const ps of this.pathStates) {
        ps.crossingWaypoints = ps.crossingWaypoints.filter(
          (cp) => !staleIds.has(cp.id),
        )
      }
    }

    // Detect new crossings introduced by re-pathing
    const nearThreshold = this.viaDiameter * 1.5
    const dirtyIndices = [...dirtyPaths]
    for (const i of dirtyIndices) {
      const pathA = this.pathStates[i]!.currentPath
      for (let j = 0; j < this.pathStates.length; j++) {
        if (i === j) continue
        const pathB = this.pathStates[j]!.currentPath
        for (let si = 0; si < pathA.length - 1; si++) {
          const a1 = pathA[si]!
          const a2 = pathA[si + 1]!
          for (let sj = 0; sj < pathB.length - 1; sj++) {
            const b1 = pathB[sj]!
            const b2 = pathB[sj + 1]!
            const intersection = getSegmentIntersection(a1, a2, b1, b2)
            if (!intersection) continue
            const pt: Point = { x: intersection.x, y: intersection.y }

            // Skip if near any existing crossing point
            const nearExisting = this.crossingPoints.some(
              (cp) => distance(pt, cp.position) < nearThreshold,
            )
            if (nearExisting) continue

            const psA = this.pathStates[i]!
            const psB = this.pathStates[j]!
            const tA = computeParametricT(psA.currentPath, pt, si)
            const tB = computeParametricT(psB.currentPath, pt, sj)

            const cp: CrossingPoint = {
              id: this.nextCrossingId++,
              pathIndexA: i,
              pathIndexB: j,
              position: pt,
              parametricTA: tA,
              parametricTB: tB,
            }

            this.crossingPoints.push(cp)
            psA.crossingWaypoints.push(cp)
            psB.crossingWaypoints.push(cp)
          }
        }
      }
    }
  }

  private finalizePhase() {
    // 1. Determine which path ducks to z=1 at each crossing and compute via
    //    entry/exit positions. Collect all via pad locations.
    const viaMargin = this.viaDiameter

    // Per-path list of via pairs: { before(z=0), viaDown(z=1), viaUp(z=1), after(z=0) }
    type ViaPair = {
      crossingId: number
      before: Point
      after: Point
      parametricT: number // for ordering along path
    }
    const pathViaPairs: ViaPair[][] = this.pathStates.map(() => [])
    const allViaPadPositions: Point[] = []

    for (const cp of this.crossingPoints) {
      const pathA = this.pathStates[cp.pathIndexA]!.currentPath
      const pathB = this.pathStates[cp.pathIndexB]!.currentPath
      const targetIdx =
        pathA.length <= pathB.length ? cp.pathIndexA : cp.pathIndexB

      const targetPath =
        targetIdx === cp.pathIndexA ? pathA : pathB

      // Find the crossing vertex on the target path
      let bestVertIdx = 0
      let bestDist = Infinity
      for (let v = 0; v < targetPath.length; v++) {
        const d = Math.hypot(
          targetPath[v]!.x - cp.position.x,
          targetPath[v]!.y - cp.position.y,
        )
        if (d < bestDist) {
          bestDist = d
          bestVertIdx = v
        }
      }

      // Travel direction through the crossing vertex
      const prev =
        bestVertIdx > 0
          ? targetPath[bestVertIdx - 1]!
          : targetPath[bestVertIdx]!
      const next =
        bestVertIdx < targetPath.length - 1
          ? targetPath[bestVertIdx + 1]!
          : targetPath[bestVertIdx]!
      const dx = next.x - prev.x
      const dy = next.y - prev.y
      const segLen = Math.hypot(dx, dy) || 1
      const ux = dx / segLen
      const uy = dy / segLen

      const beforePt: Point = {
        x: cp.position.x - ux * viaMargin,
        y: cp.position.y - uy * viaMargin,
      }
      const afterPt: Point = {
        x: cp.position.x + ux * viaMargin,
        y: cp.position.y + uy * viaMargin,
      }

      const t =
        targetIdx === cp.pathIndexA ? cp.parametricTA : cp.parametricTB
      pathViaPairs[targetIdx]!.push({
        crossingId: cp.id,
        before: beforePt,
        after: afterPt,
        parametricT: t,
      })

      // Collect via pad positions (both entry and exit) as obstacles
      allViaPadPositions.push(beforePt, afterPt)
    }

    // 2. Build a new mesh that includes via pads as obstacles so other traces
    //    route around them.
    const obstacleMargin = this.srj.defaultObstacleMargin ?? this.minTraceWidth
    const expandedObstacles = this.srj.obstacles.map((obs) =>
      rectToPolygon(
        obs.center.x,
        obs.center.y,
        obs.width,
        obs.height,
        obstacleMargin,
      ),
    )
    // Add via pads as small square obstacles
    const viaPadSize = this.viaDiameter
    for (const vp of allViaPadPositions) {
      expandedObstacles.push(
        rectToPolygon(vp.x, vp.y, viaPadSize, viaPadSize, obstacleMargin),
      )
    }

    const mergedObstacles = mergeOverlappingRects(expandedObstacles)
    const cdtResult = cdtTriangulate({
      bounds: this.srj.bounds,
      obstacles: mergedObstacles,
    })
    const rawMesh = buildMeshFromRegions(cdtResult)
    const viaMesh = mergeMesh(rawMesh)
    const finalSearch = new SearchInstance(viaMesh)

    // 3. Re-pathfind each trace through the new mesh.
    //    Each path has two kinds of crossing waypoints:
    //    - Via crossings (this path ducks to z=1): route through before/after
    //    - Pass-through crossings (other path ducks): route through the point on z=0
    //    Build a sorted event list and chain-search between them.

    // Track which crossings have vias on which path
    const viaOnPath = new Set<string>() // "crossingId:pathIndex"
    for (let i = 0; i < pathViaPairs.length; i++) {
      for (const vp of pathViaPairs[i]!) {
        viaOnPath.add(`${vp.crossingId}:${i}`)
      }
    }

    const adjustedPaths: Array<{ x: number; y: number; z: number }[]> = []

    for (let i = 0; i < this.pathStates.length; i++) {
      const ps = this.pathStates[i]!

      // Build sorted event list for this path
      type PathEvent =
        | { type: "via"; before: Point; after: Point; t: number }
        | { type: "passthrough"; position: Point; t: number }

      const events: PathEvent[] = []

      for (const cp of ps.crossingWaypoints) {
        const t =
          cp.pathIndexA === i ? cp.parametricTA : cp.parametricTB
        if (viaOnPath.has(`${cp.id}:${i}`)) {
          // This path ducks at this crossing
          const vp = pathViaPairs[i]!.find((v) => v.crossingId === cp.id)!
          events.push({ type: "via", before: vp.before, after: vp.after, t })
        } else {
          // Other path ducks — this path passes through on z=0
          events.push({ type: "passthrough", position: cp.position, t })
        }
      }

      events.sort((a, b) => a.t - b.t)

      if (events.length === 0) {
        // No crossings at all — simple re-pathfind
        const path = this.chainSearch(
          finalSearch,
          [ps.originalStart, ps.originalEnd],
          viaMesh,
        )
        adjustedPaths.push(path.map((p) => ({ x: p.x, y: p.y, z: 0 })))
        continue
      }

      const route: Array<{ x: number; y: number; z: number }> = []
      let cursor: Point = ps.originalStart

      for (let e = 0; e < events.length; e++) {
        const ev = events[e]!

        if (ev.type === "passthrough") {
          // z=0 segment: cursor → passthrough point
          const leg = this.chainSearch(
            finalSearch,
            [cursor, ev.position],
            viaMesh,
          )
          const startK = route.length > 0 ? 1 : 0
          for (let k = startK; k < leg.length; k++) {
            route.push({ x: leg[k]!.x, y: leg[k]!.y, z: 0 })
          }
          cursor = ev.position
        } else {
          // z=0 segment: cursor → via before
          const leg = this.chainSearch(
            finalSearch,
            [cursor, ev.before],
            viaMesh,
          )
          const startK = route.length > 0 ? 1 : 0
          for (let k = startK; k < leg.length; k++) {
            route.push({ x: leg[k]!.x, y: leg[k]!.y, z: 0 })
          }

          // Via transition: z=0 → z=1 → z=1 → z=0
          const lastPt = route[route.length - 1]!
          route.push({ x: lastPt.x, y: lastPt.y, z: 1 })
          route.push({ x: ev.after.x, y: ev.after.y, z: 1 })
          route.push({ x: ev.after.x, y: ev.after.y, z: 0 })
          cursor = ev.after
        }
      }

      // Final z=0 segment: cursor → end
      const lastLeg = this.chainSearch(
        finalSearch,
        [cursor, ps.originalEnd],
        viaMesh,
      )
      for (let k = 1; k < lastLeg.length; k++) {
        route.push({ x: lastLeg[k]!.x, y: lastLeg[k]!.y, z: 0 })
      }

      adjustedPaths.push(route)
    }

    // 4. Build resolved paths with via lists
    this.resolvedPaths = this.pathStates.map((ps, i) => {
      const route = adjustedPaths[i]!
      const vias: Array<{ x: number; y: number }> = []
      for (let k = 1; k < route.length; k++) {
        if (route[k]!.z !== route[k - 1]!.z) {
          vias.push({ x: route[k]!.x, y: route[k]!.y })
        }
      }
      return {
        connectionName: ps.connectionName,
        route,
        vias,
      }
    })

    this.solved = true
  }

  /** Pathfind through a sequence of waypoints, concatenating segments. */
  private chainSearch(
    searchInstance: SearchInstance,
    waypoints: Point[],
    mesh: Mesh,
  ): Point[] {
    const result: Point[] = []
    for (let i = 0; i < waypoints.length - 1; i++) {
      let from = waypoints[i]!
      let to = waypoints[i + 1]!

      // Snap endpoints to mesh if they landed inside a via obstacle
      const fromLoc = mesh.getPointLocation(from)
      if (fromLoc.type === PointLocationType.NOT_ON_MESH) {
        // Find nearest mesh point by binary search from midpoint
        const mid: Point = {
          x: (from.x + to.x) / 2,
          y: (from.y + to.y) / 2,
        }
        from = snapToMesh(mid, from, mesh)
      }
      const toLoc = mesh.getPointLocation(to)
      if (toLoc.type === PointLocationType.NOT_ON_MESH) {
        const mid: Point = {
          x: (from.x + to.x) / 2,
          y: (from.y + to.y) / 2,
        }
        to = snapToMesh(mid, to, mesh)
      }

      searchInstance.setStartGoal(from, to)
      const found = searchInstance.search()
      if (found) {
        const pts = searchInstance.getPathPoints()
        if (i > 0 && pts.length > 0) {
          result.push(...pts.slice(1))
        } else {
          result.push(...pts)
        }
      } else {
        // Fallback: direct line
        if (i > 0) {
          result.push(to)
        } else {
          result.push(from, to)
        }
      }
    }
    return result
  }

  getResolvedPaths(): ResolvedPath[] {
    return this.resolvedPaths
  }

  visualize(): GraphicsObject {
    const lines: GraphicsObject["lines"] = []
    const circles: GraphicsObject["circles"] = []
    const points: GraphicsObject["points"] = []

    if (this.phase === "FINALIZE" || this.solved) {
      for (const resolved of this.resolvedPaths) {
        const color = this.colorMap[resolved.connectionName] ?? "green"
        if (resolved.route.length > 1) {
          lines.push({
            points: resolved.route.map((p) => ({ x: p.x, y: p.y })),
            strokeColor: color,
            strokeWidth: 0.05,
          })
        }
        for (const via of resolved.vias) {
          circles.push({
            center: via,
            radius: this.viaDiameter / 2,
            fill: "rgba(255,165,0,0.5)",
            stroke: "orange",
          })
        }
      }
    } else {
      // Draw current paths during repulsion
      for (const ps of this.pathStates) {
        const color = this.colorMap[ps.connectionName] ?? "green"
        if (ps.currentPath.length > 1) {
          lines.push({
            points: ps.currentPath.map((p) => ({ x: p.x, y: p.y })),
            strokeColor: color,
            strokeWidth: 0.05,
          })
        }
      }
    }

    // Draw crossing points as circles
    for (const cp of this.crossingPoints) {
      circles.push({
        center: cp.position,
        radius: this.viaDiameter * 0.5,
        fill: "rgba(255,255,0,0.3)",
        stroke: "rgba(255,165,0,0.8)",
      })
      points.push({
        x: cp.position.x,
        y: cp.position.y,
        color: "red",
        label: `x${cp.id}`,
      })
    }

    return { lines, circles, points }
  }
}
