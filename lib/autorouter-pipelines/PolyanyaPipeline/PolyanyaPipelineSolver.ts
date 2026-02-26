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
import { CrossingResolverSolver } from "../../solvers/PolyanyaSolver/CrossingResolverSolver"
import { PolyanyaOutputSolver } from "../../solvers/PolyanyaSolver/PolyanyaOutputSolver"

type PipelineStep<T extends new (...args: any[]) => BaseSolver> = {
  solverName: string
  solverClass: T
  getConstructorParams: (
    instance: PolyanyaPipelineSolver,
  ) => ConstructorParameters<T>
  onSolved?: (instance: PolyanyaPipelineSolver) => void
}

function definePipelineStep<
  T extends new (...args: any[]) => BaseSolver,
  const P extends ConstructorParameters<T>,
>(
  solverName: keyof PolyanyaPipelineSolver,
  solverClass: T,
  getConstructorParams: (instance: PolyanyaPipelineSolver) => P,
  opts: {
    onSolved?: (instance: PolyanyaPipelineSolver) => void
  } = {},
): PipelineStep<T> {
  return {
    solverName,
    solverClass,
    getConstructorParams,
    onSolved: opts.onSolved,
  }
}

export class PolyanyaPipelineSolver extends BaseSolver {
  override getSolverName(): string {
    return "PolyanyaPipelineSolver"
  }

  netToPointPairsSolver?: NetToPointPairsSolver2_OffBoardConnection
  meshSolver?: PolyanyaMeshSolver
  pathSolver?: PolyanyaPathSolver
  crossingResolver?: CrossingResolverSolver
  outputSolver?: PolyanyaOutputSolver

  colorMap: Record<string, string>
  connMap: ConnectivityMap
  srjWithPointPairs?: SimpleRouteJson
  viaDiameter: number
  minTraceWidth: number

  startTimeOfPhase: Record<string, number> = {}
  endTimeOfPhase: Record<string, number> = {}
  timeSpentOnPhase: Record<string, number> = {}

  activeSubSolver?: BaseSolver | null = null

  pipelineDef = [
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
    definePipelineStep(
      "meshSolver",
      PolyanyaMeshSolver,
      (pps) => [
        pps.srjWithPointPairs ?? pps.srj,
        pps.srj.defaultObstacleMargin ?? pps.minTraceWidth,
      ],
    ),
    definePipelineStep(
      "pathSolver",
      PolyanyaPathSolver,
      (pps) => [
        {
          mesh: pps.meshSolver!.getMesh(),
          srj: pps.srjWithPointPairs ?? pps.srj,
          colorMap: pps.colorMap,
          minTraceWidth: pps.minTraceWidth,
        },
      ],
    ),
    definePipelineStep(
      "crossingResolver",
      CrossingResolverSolver,
      (pps) => [
        {
          paths: pps.pathSolver!.getResults(),
          srj: pps.srjWithPointPairs ?? pps.srj,
          colorMap: pps.colorMap,
          minTraceWidth: pps.minTraceWidth,
          layerCount: pps.srj.layerCount,
          viaDiameter: pps.viaDiameter,
        },
      ],
    ),
    definePipelineStep(
      "outputSolver",
      PolyanyaOutputSolver,
      (pps) => [
        {
          resolvedPaths: pps.crossingResolver!.getResolvedPaths(),
          srj: pps.srjWithPointPairs ?? pps.srj,
          minTraceWidth: pps.minTraceWidth,
          viaDiameter: pps.viaDiameter,
        },
      ],
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
    return {
      ...this.srj,
      traces: this.getOutputSimplifiedPcbTraces(),
    }
  }

  visualize(): GraphicsObject {
    if (!this.solved && this.activeSubSolver)
      return this.activeSubSolver.visualize()

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
    if (this.pathSolver) {
      const lines: Line[] = []
      for (const result of this.pathSolver.results) {
        if (result.path.length > 1) {
          lines.push({
            points: result.path.map((p) => ({ x: p.x, y: p.y })),
            strokeColor: this.colorMap[result.connectionName],
          })
        }
        if (lines.length > 200) break
      }
      return { lines }
    }
    return {}
  }
}
