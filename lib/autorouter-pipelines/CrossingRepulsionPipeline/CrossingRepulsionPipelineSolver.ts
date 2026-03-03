import type { GraphicsObject, Line } from "graphics-debug"
import { combineVisualizations } from "../../utils/combineVisualizations"
import type {
  SimpleRouteJson,
  SimplifiedPcbTrace,
  SimplifiedPcbTraces,
} from "../../types"
import { BaseSolver } from "../../solvers/BaseSolver"
import { getColorMap } from "../../solvers/colors"
import { ConnectivityMap } from "circuit-json-to-connectivity-map"
import { getConnectivityMapFromSimpleRouteJson } from "lib/utils/getConnectivityMapFromSimpleRouteJson"
import { NetToPointPairsSolver2_OffBoardConnection } from "../../solvers/NetToPointPairsSolver2_OffBoardConnection/NetToPointPairsSolver2_OffBoardConnection"
import { convertSrjToGraphicsObject } from "lib/utils/convertSrjToGraphicsObject"
import { PolyanyaMeshSolver } from "../../solvers/PolyanyaSolver/PolyanyaMeshSolver"
import { PolyanyaPathSolver } from "../../solvers/PolyanyaSolver/PolyanyaPathSolver"
import { CrossingRepulsionSolver } from "../../solvers/PolyanyaSolver/CrossingRepulsionSolver"
import { PolyanyaOutputSolver } from "../../solvers/PolyanyaSolver/PolyanyaOutputSolver"
import { GreedySequentialPathSolver } from "../../solvers/PolyanyaSolver/GreedySequentialPathSolver"

type PipelineStep<T extends new (...args: any[]) => BaseSolver> = {
  solverName: string
  solverClass: T
  getConstructorParams: (
    instance: CrossingRepulsionPipelineSolver,
  ) => ConstructorParameters<T>
  onSolved?: (instance: CrossingRepulsionPipelineSolver) => void
}

function definePipelineStep<
  T extends new (...args: any[]) => BaseSolver,
  const P extends ConstructorParameters<T>,
>(
  solverName: keyof CrossingRepulsionPipelineSolver,
  solverClass: T,
  getConstructorParams: (instance: CrossingRepulsionPipelineSolver) => P,
  opts: {
    onSolved?: (instance: CrossingRepulsionPipelineSolver) => void
  } = {},
): PipelineStep<T> {
  return {
    solverName,
    solverClass,
    getConstructorParams,
    onSolved: opts.onSolved,
  }
}

export class CrossingRepulsionPipelineSolver extends BaseSolver {
  override getSolverName(): string {
    return "CrossingRepulsionPipelineSolver"
  }

  netToPointPairsSolver?: NetToPointPairsSolver2_OffBoardConnection
  greedySolver?: GreedySequentialPathSolver
  meshSolver?: PolyanyaMeshSolver
  pathSolver?: PolyanyaPathSolver
  crossingResolver?: CrossingRepulsionSolver
  outputSolver?: PolyanyaOutputSolver

  colorMap: Record<string, string>
  connMap: ConnectivityMap
  srjWithPointPairs?: SimpleRouteJson
  /** SRJ with only the unrouted connections (for fallback pipeline) */
  srjForFallback?: SimpleRouteJson
  viaDiameter: number
  minTraceWidth: number

  startTimeOfPhase: Record<string, number> = {}
  endTimeOfPhase: Record<string, number> = {}
  timeSpentOnPhase: Record<string, number> = {}

  activeSubSolver?: BaseSolver | null = null

  pipelineDef = [
    // Step 1: Net-to-point-pairs
    definePipelineStep(
      "netToPointPairsSolver",
      NetToPointPairsSolver2_OffBoardConnection,
      (pps) => [pps.srj, pps.colorMap],
      {
        onSolved: (pps) => {
          pps.srjWithPointPairs =
            pps.netToPointPairsSolver?.getNewSimpleRouteJson()
          pps.colorMap = getColorMap(pps.srjWithPointPairs!, pps.connMap)
          pps.connMap = getConnectivityMapFromSimpleRouteJson(
            pps.srjWithPointPairs!,
          )
        },
      },
    ),
    // Step 2: Greedy sequential solver — routes as many traces as it can
    definePipelineStep(
      "greedySolver",
      GreedySequentialPathSolver,
      (pps) => [
        {
          srj: pps.srjWithPointPairs ?? pps.srj,
          colorMap: pps.colorMap,
          minTraceWidth: pps.minTraceWidth,
          margin: pps.srj.defaultObstacleMargin ?? pps.minTraceWidth,
        },
      ],
      {
        onSolved: (pps) => {
          // Build a reduced SRJ with only unrouted connections for fallback
          const unrouted = new Set(pps.greedySolver!.getUnroutedConnectionNames())
          if (unrouted.size === 0) {
            // Everything routed — skip the fallback pipeline
            pps.srjForFallback = undefined
          } else {
            const baseSrj = pps.srjWithPointPairs ?? pps.srj
            pps.srjForFallback = {
              ...baseSrj,
              connections: baseSrj.connections.filter((c) => unrouted.has(c.name)),
            }
          }
        },
      },
    ),
    // Step 3: Mesh solver for unrouted connections (skipped if all routed)
    definePipelineStep(
      "meshSolver",
      PolyanyaMeshSolver,
      (pps) => [
        pps.srjForFallback ?? pps.srjWithPointPairs ?? pps.srj,
        pps.srj.defaultObstacleMargin ?? pps.minTraceWidth,
      ],
    ),
    // Step 4: Path solver for unrouted connections
    definePipelineStep(
      "pathSolver",
      PolyanyaPathSolver,
      (pps) => [
        {
          mesh: pps.meshSolver!.getMesh(),
          srj: pps.srjForFallback ?? pps.srjWithPointPairs ?? pps.srj,
          colorMap: pps.colorMap,
          minTraceWidth: pps.minTraceWidth,
        },
      ],
    ),
    // Step 5: Crossing repulsion for unrouted connections
    definePipelineStep(
      "crossingResolver",
      CrossingRepulsionSolver,
      (pps) => [
        {
          paths: pps.pathSolver!.getResults(),
          mesh: pps.meshSolver!.getMesh(),
          srj: pps.srjForFallback ?? pps.srjWithPointPairs ?? pps.srj,
          colorMap: pps.colorMap,
          minTraceWidth: pps.minTraceWidth,
          layerCount: pps.srj.layerCount,
          viaDiameter: pps.viaDiameter,
        },
      ],
    ),
    // Step 6: Output — merge greedy results + crossing repulsion results
    definePipelineStep(
      "outputSolver",
      PolyanyaOutputSolver,
      (pps) => {
        const baseSrj = pps.srjWithPointPairs ?? pps.srj
        const greedyPaths = pps.greedySolver?.getResolvedPaths() ?? []
        const fallbackPaths = pps.crossingResolver?.getResolvedPaths() ?? []
        const allPaths = [...greedyPaths, ...fallbackPaths]
        const effectiveLayerCount = pps.greedySolver?.getEffectiveLayerCount() ?? pps.srj.layerCount
        return [
          {
            resolvedPaths: allPaths,
            srj: baseSrj.layerCount >= effectiveLayerCount
              ? baseSrj
              : { ...baseSrj, layerCount: effectiveLayerCount },
            minTraceWidth: pps.minTraceWidth,
            viaDiameter: pps.viaDiameter,
          },
        ]
      },
    ),
  ]

  constructor(
    public readonly srj: SimpleRouteJson,
    public readonly opts: { effort?: number } = {},
  ) {
    super()
    this.MAX_ITERATIONS = 100e6
    this.viaDiameter = srj.minViaDiameter ?? 0.6
    this.minTraceWidth = srj.minTraceWidth
    this.connMap = getConnectivityMapFromSimpleRouteJson(srj)
    this.colorMap = getColorMap(srj, this.connMap)
  }

  getConstructorParams() {
    return [this.srj, this.opts] as const
  }

  currentPipelineStepIndex = 0

  _step() {
    const pipelineStepDef = this.pipelineDef[this.currentPipelineStepIndex]
    if (!pipelineStepDef) {
      this.solved = true
      return
    }

    // Skip fallback steps (mesh/path/crossingResolver) if greedy routed everything
    if (
      !this.srjForFallback &&
      this.greedySolver &&
      (pipelineStepDef.solverName === "meshSolver" ||
        pipelineStepDef.solverName === "pathSolver" ||
        pipelineStepDef.solverName === "crossingResolver")
    ) {
      this.currentPipelineStepIndex++
      return
    }

    if (this.activeSubSolver) {
      this.activeSubSolver.step()
      if (this.activeSubSolver.solved) {
        this.endTimeOfPhase[pipelineStepDef.solverName] = performance.now()
        this.timeSpentOnPhase[pipelineStepDef.solverName] =
          this.endTimeOfPhase[pipelineStepDef.solverName]! -
          this.startTimeOfPhase[pipelineStepDef.solverName]!
        pipelineStepDef.onSolved?.(this)
        this.activeSubSolver = null
        this.currentPipelineStepIndex++
      } else if (this.activeSubSolver.failed) {
        this.error = this.activeSubSolver?.error
        this.failed = true
        this.activeSubSolver = null
      }
      return
    }

    const constructorParams = pipelineStepDef.getConstructorParams(this)
    // @ts-ignore
    this.activeSubSolver = new pipelineStepDef.solverClass(...constructorParams)
    ;(this as any)[pipelineStepDef.solverName] = this.activeSubSolver
    this.timeSpentOnPhase[pipelineStepDef.solverName] = 0
    this.startTimeOfPhase[pipelineStepDef.solverName] = performance.now()
  }

  getCurrentPhase(): string {
    return this.pipelineDef[this.currentPipelineStepIndex]?.solverName ?? "none"
  }

  getOutputSimplifiedPcbTraces(): SimplifiedPcbTraces {
    if (!this.solved || !this.outputSolver) {
      throw new Error("Cannot get output before solving is complete")
    }
    return this.outputSolver.getSimplifiedTraces()
  }

  getOutputSimpleRouteJson(): SimpleRouteJson {
    const effectiveLayerCount = this.greedySolver?.getEffectiveLayerCount() ?? this.srj.layerCount
    return {
      ...this.srj,
      layerCount: Math.max(this.srj.layerCount, effectiveLayerCount),
      traces: this.getOutputSimplifiedPcbTraces(),
    }
  }

  visualize(): GraphicsObject {
    if (!this.solved && this.activeSubSolver)
      return this.activeSubSolver.visualize()

    const greedyViz = this.greedySolver?.visualize()
    const meshViz = this.meshSolver?.visualize()
    const pathViz = this.pathSolver?.visualize()
    const crossingViz = this.crossingResolver?.visualize()
    const outputViz = this.outputSolver?.visualize()

    const { minX, maxX, minY, maxY } = this.srj.bounds
    const problemLines: Line[] = [
      {
        points: [
          { x: minX, y: minY },
          { x: maxX, y: minY },
          { x: maxX, y: maxY },
          { x: minX, y: maxY },
          { x: minX, y: minY },
        ],
        strokeColor: "rgba(255,0,0,0.25)",
      },
    ]

    const problemViz: GraphicsObject = {
      points: this.srj.connections.flatMap((c) =>
        c.pointsToConnect.map((p) => ({
          ...p,
          label: `${c.name} ${p.pcb_port_id ?? ""}`,
        })),
      ),
      rects: (this.srj.obstacles ?? []).map((o) => ({
        ...o,
        fill: o.layers?.includes("top")
          ? "rgba(255,0,0,0.25)"
          : o.layers?.includes("bottom")
            ? "rgba(0,0,255,0.25)"
            : "rgba(255,0,0,0.25)",
        label: o.layers?.join(", "),
      })),
      lines: problemLines,
    }

    const visualizations = [
      problemViz,
      greedyViz,
      meshViz,
      pathViz,
      crossingViz,
      outputViz,
      this.solved
        ? combineVisualizations(
            problemViz,
            convertSrjToGraphicsObject(this.getOutputSimpleRouteJson()),
          )
        : null,
    ].filter(Boolean) as GraphicsObject[]

    return combineVisualizations(...visualizations)
  }

  preview(): GraphicsObject {
    if (this.greedySolver) {
      const lines: Line[] = []
      for (const rp of this.greedySolver.getResolvedPaths()) {
        if (rp.route.length > 1) {
          lines.push({
            points: rp.route.map((p) => ({ x: p.x, y: p.y })),
            strokeColor: this.colorMap[rp.connectionName],
          })
        }
        if (lines.length > 200) break
      }
      return { lines }
    }
    return {}
  }
}
