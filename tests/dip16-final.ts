import { GreedySequentialPathSolver } from "../lib/solvers/PolyanyaSolver/GreedySequentialPathSolver"
import { NetToPointPairsSolver2_OffBoardConnection } from "../lib/solvers/NetToPointPairsSolver2_OffBoardConnection/NetToPointPairsSolver2_OffBoardConnection"
import { getConnectivityMapFromSimpleRouteJson } from "../lib/utils/getConnectivityMapFromSimpleRouteJson"
import { getColorMap } from "../lib/solvers/colors"
import reproJson from "./repro/dip16-crossing-traces.json"

const baseSrj = reproJson as any
const connMap = getConnectivityMapFromSimpleRouteJson(baseSrj)
const colorMap = getColorMap(baseSrj, connMap)
const pairSolver = new NetToPointPairsSolver2_OffBoardConnection(baseSrj, colorMap)
pairSolver.solve()
const srj = pairSolver.getNewSimpleRouteJson()
const connMap2 = getConnectivityMapFromSimpleRouteJson(srj)
const colorMap2 = getColorMap(srj, connMap2)

for (const toggle of [false, true]) {
  const t0 = performance.now()
  const solver = new GreedySequentialPathSolver({
    srj, colorMap: colorMap2, minTraceWidth: srj.minTraceWidth,
    margin: srj.defaultObstacleMargin ?? srj.minTraceWidth,
    useOccupancyToggle: toggle,
  })
  solver.solve()
  const ms = performance.now() - t0
  const v = solver.validationResult!
  const mode = toggle ? "TOGGLE" : "CDT   "
  console.log(`${mode}: ${v.routedConnections}/${v.totalConnections}  crossings:${v.crossNetCrossings.length}  ${ms.toFixed(0)}ms`)
}
