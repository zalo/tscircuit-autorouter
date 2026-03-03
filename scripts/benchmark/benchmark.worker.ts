import { runAllChecks } from "@tscircuit/checks"
import * as autorouterModule from "../../lib"
import { convertToCircuitJson } from "../../lib/testing/utils/convertToCircuitJson"
import type {
  SimpleRouteJson,
  SimplifiedPcbTrace,
} from "../../lib/types/srj-types"

type BenchmarkTask = {
  solverName: string
  scenarioName: string
  scenario: SimpleRouteJson
}

type BenchmarkResult = {
  scenarioName: string
  elapsedTimeMs: number
  didSolve: boolean
  relaxedDrcPassed: boolean
  error?: string
}

type SolverInstance = {
  solved?: boolean
  failed?: boolean
  srjWithPointPairs?: SimpleRouteJson
  solve: () => void
  getOutputSimplifiedPcbTraces?: () => SimplifiedPcbTrace[]
}

const getSolverConstructor = (solverName: string) => {
  const ctor = (autorouterModule as Record<string, unknown>)[solverName]
  if (typeof ctor !== "function") {
    throw new Error(`Solver \"${solverName}\" was not found`)
  }
  return ctor as new (
    srj: SimpleRouteJson,
  ) => SolverInstance
}

const hasTraceError = (error: unknown): boolean => {
  if (!error || typeof error !== "object") {
    return false
  }
  if (!("error_type" in error)) {
    return false
  }
  return (error as { error_type?: string }).error_type === "pcb_trace_error"
}

const runTask = async (task: BenchmarkTask): Promise<BenchmarkResult> => {
  const SolverConstructor = getSolverConstructor(task.solverName)
  const solver = new SolverConstructor(task.scenario)
  const start = performance.now()

  try {
    solver.solve()
  } catch {
    solver.solved = false
  }

  const elapsedTimeMs = performance.now() - start
  const didSolve = Boolean(solver.solved)

  if (!didSolve) {
    return {
      scenarioName: task.scenarioName,
      elapsedTimeMs,
      didSolve,
      relaxedDrcPassed: false,
    }
  }

  try {
    const traces = solver.failed
      ? []
      : (solver.getOutputSimplifiedPcbTraces?.() ?? [])
    const circuitJson = convertToCircuitJson(
      solver.srjWithPointPairs ?? task.scenario,
      traces,
      task.scenario.minTraceWidth,
      task.scenario.minViaDiameter,
    )
    const checks = await runAllChecks(circuitJson)
    const relaxedDrcPassed = !checks.some(hasTraceError)

    return {
      scenarioName: task.scenarioName,
      elapsedTimeMs,
      didSolve,
      relaxedDrcPassed,
    }
  } catch (error) {
    return {
      scenarioName: task.scenarioName,
      elapsedTimeMs,
      didSolve,
      relaxedDrcPassed: false,
      error: error instanceof Error ? error.message : String(error),
    }
  }
}

self.onmessage = async (event: MessageEvent<BenchmarkTask>) => {
  const result = await runTask(event.data)
  self.postMessage(result)
}
