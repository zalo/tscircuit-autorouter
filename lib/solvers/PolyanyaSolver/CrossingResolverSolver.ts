import type { GraphicsObject } from "graphics-debug"
import { getSegmentIntersection } from "@tscircuit/math-utils"
import { BaseSolver } from "../BaseSolver"
import type { SimpleRouteJson } from "../../types"
import type {
  PolyanyaPathResult,
  Crossing,
  ResolvedPath,
} from "./types"

export class CrossingResolverSolver extends BaseSolver {
  paths: PolyanyaPathResult[]
  srj: SimpleRouteJson
  colorMap: Record<string, string>
  minTraceWidth: number
  layerCount: number
  viaDiameter: number

  crossings: Crossing[] = []
  resolvedPaths: ResolvedPath[] = []

  private phase = 0

  constructor(params: {
    paths: PolyanyaPathResult[]
    srj: SimpleRouteJson
    colorMap: Record<string, string>
    minTraceWidth: number
    layerCount: number
    viaDiameter: number
  }) {
    super()
    this.paths = params.paths
    this.srj = params.srj
    this.colorMap = params.colorMap
    this.minTraceWidth = params.minTraceWidth
    this.layerCount = params.layerCount
    this.viaDiameter = params.viaDiameter
    this.MAX_ITERATIONS = 100
  }

  _step() {
    switch (this.phase) {
      case 0:
        this.detectCrossings()
        this.phase = 1
        break
      case 1:
        this.resolveAllCrossings()
        this.phase = 2
        break
      case 2:
        this.buildOutput()
        this.solved = true
        break
    }
    this.progress = this.phase / 3
  }

  /** Phase 0: Detect all pairwise segment crossings */
  private detectCrossings() {
    for (let i = 0; i < this.paths.length; i++) {
      const pathA = this.paths[i]!
      for (let j = i + 1; j < this.paths.length; j++) {
        const pathB = this.paths[j]!
        for (let si = 0; si < pathA.path.length - 1; si++) {
          const a1 = pathA.path[si]!
          const a2 = pathA.path[si + 1]!
          for (let sj = 0; sj < pathB.path.length - 1; sj++) {
            const b1 = pathB.path[sj]!
            const b2 = pathB.path[sj + 1]!
            const intersection = getSegmentIntersection(a1, a2, b1, b2)
            if (intersection) {
              const angleA = Math.atan2(a2.y - a1.y, a2.x - a1.x)
              const angleB = Math.atan2(b2.y - b1.y, b2.x - b1.x)
              let angleDiff = Math.abs(angleA - angleB)
              if (angleDiff > Math.PI) angleDiff = 2 * Math.PI - angleDiff
              if (angleDiff > Math.PI / 2)
                angleDiff = Math.PI - angleDiff

              this.crossings.push({
                pathIndexA: i,
                pathIndexB: j,
                segIndexA: si,
                segIndexB: sj,
                point: { x: intersection.x, y: intersection.y },
                angle: angleDiff,
              })
            }
          }
        }
      }
    }
  }

  /** Phase 1: Resolve ALL crossings with via insertion */
  private resolveAllCrossings() {
    // Start with original paths on z=0
    const adjustedPaths: Array<{ x: number; y: number; z: number }[]> =
      this.paths.map((p) => p.path.map((pt) => ({ x: pt.x, y: pt.y, z: 0 })))

    // For each crossing, move the shorter path to layer 1 at the crossing zone.
    // A path may get multiple vias if it's involved in multiple crossings —
    // each crossing is resolved independently.
    if (this.layerCount >= 2) {
      for (const crossing of this.crossings) {
        const pathA = adjustedPaths[crossing.pathIndexA]!
        const pathB = adjustedPaths[crossing.pathIndexB]!

        // Choose shorter path to transition
        const lenA = pathA.length
        const lenB = pathB.length
        const targetIdx =
          lenA <= lenB ? crossing.pathIndexA : crossing.pathIndexB

        const targetPath = adjustedPaths[targetIdx]!
        const cp = crossing.point

        // Find the segment closest to the crossing point
        let bestSegIdx = 0
        let bestDist = Infinity
        for (let s = 0; s < targetPath.length - 1; s++) {
          const mx = (targetPath[s]!.x + targetPath[s + 1]!.x) / 2
          const my = (targetPath[s]!.y + targetPath[s + 1]!.y) / 2
          const d = Math.hypot(mx - cp.x, my - cp.y)
          if (d < bestDist) {
            bestDist = d
            bestSegIdx = s
          }
        }

        // Insert via-cross-via pattern around the crossing
        const viaMargin = this.viaDiameter
        const segStart = targetPath[bestSegIdx]!
        const segEnd = targetPath[bestSegIdx + 1]!
        const dx = segEnd.x - segStart.x
        const dy = segEnd.y - segStart.y
        const segLen = Math.hypot(dx, dy) || 1
        const ux = dx / segLen
        const uy = dy / segLen

        // Insert points: before crossing on z=0, via to z=1, cross on z=1, via back to z=0
        const before = {
          x: cp.x - ux * viaMargin,
          y: cp.y - uy * viaMargin,
          z: 0,
        }
        const after = {
          x: cp.x + ux * viaMargin,
          y: cp.y + uy * viaMargin,
          z: 0,
        }
        const crossStart = { x: before.x, y: before.y, z: 1 }
        const crossEnd = { x: after.x, y: after.y, z: 1 }

        // Rebuild the path with via transitions
        const newPath: Array<{ x: number; y: number; z: number }> = []
        for (let p = 0; p <= bestSegIdx; p++) {
          newPath.push(targetPath[p]!)
        }
        newPath.push(before, crossStart, crossEnd, after)
        for (let p = bestSegIdx + 1; p < targetPath.length; p++) {
          newPath.push(targetPath[p]!)
        }
        adjustedPaths[targetIdx] = newPath
      }
    }

    // Store adjusted paths for output
    this.resolvedPaths = this.paths.map((p, i) => {
      const route = adjustedPaths[i]!
      const vias: Array<{ x: number; y: number }> = []

      // Detect vias (z-level changes)
      for (let k = 1; k < route.length; k++) {
        if (route[k]!.z !== route[k - 1]!.z) {
          vias.push({ x: route[k]!.x, y: route[k]!.y })
        }
      }

      return {
        connectionName: p.connectionName,
        route,
        vias,
      }
    })
  }

  /** Phase 2: Build final output */
  private buildOutput() {
    // resolvedPaths already built in phase 1
  }

  getResolvedPaths(): ResolvedPath[] {
    return this.resolvedPaths
  }

  visualize(): GraphicsObject {
    const lines: GraphicsObject["lines"] = []
    const circles: GraphicsObject["circles"] = []
    const points: GraphicsObject["points"] = []

    // Draw resolved paths
    for (const resolved of this.resolvedPaths) {
      const color = this.colorMap[resolved.connectionName] ?? "green"
      if (resolved.route.length > 1) {
        lines.push({
          points: resolved.route.map((p) => ({ x: p.x, y: p.y })),
          strokeColor: color,
          strokeWidth: 0.05,
        })
      }

      // Via markers
      for (const via of resolved.vias) {
        circles.push({
          center: via,
          radius: this.viaDiameter / 2,
          fill: "rgba(255,165,0,0.5)",
          stroke: "orange",
        })
      }
    }

    // Draw crossing points
    for (const crossing of this.crossings) {
      points.push({
        x: crossing.point.x,
        y: crossing.point.y,
        color: "red",
        label: "cross",
      })
    }

    return { lines, circles, points }
  }
}
