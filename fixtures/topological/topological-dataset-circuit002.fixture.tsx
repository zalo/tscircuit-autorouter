import { AutoroutingPipelineDebugger } from "lib/testing/AutoroutingPipelineDebugger"
import { TopologicalPipelineSolver } from "lib/autorouter-pipelines/TopologicalPipeline/TopologicalPipelineSolver"
import * as dataset from "@tscircuit/autorouting-dataset-01"

const circuits = (dataset as any).default ?? dataset
const srj = (circuits as any)["circuit002"]

export default () => (
  <AutoroutingPipelineDebugger
    createSolver={(s) => new TopologicalPipelineSolver(s)}
    srj={srj}
  />
)
