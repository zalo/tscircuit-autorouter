import { InteractiveGraphics } from "graphics-debug/react"
import { GreedyDescentCrossingViasSolver } from "lib/solvers/HighDensitySolver/GreedyDescentCrossingViasSolver"
import { useState, useMemo } from "react"

const nodeWithPortPoints = {
  capacityMeshNodeId: "cmn_2",
  portPoints: [
    { x: -2.6499999999999995, y: 6.985, z: 0, connectionName: "source_trace_0" },
    { x: -2.6499999999999986, y: -4.476206896551725, z: 3, connectionName: "source_trace_13" },
    { x: -20.75, y: 13.97, z: 1, connectionName: "source_trace_13" },
    { x: -20.75, y: 11.429999999999998, z: 1, connectionName: "source_trace_15" },
    { x: -2.6499999999999995, y: 1.9050000000000002, z: 0, connectionName: "source_trace_4" },
    { x: -2.6499999999999986, y: 1.141379310344828, z: 3, connectionName: "source_trace_4" },
    { x: -20.75, y: 8.889999999999999, z: 1, connectionName: "source_trace_17" },
    { x: -2.6499999999999995, y: 0.6350000000000007, z: 0, connectionName: "source_trace_5" },
    { x: -2.6499999999999995, y: -1.905, z: 0, connectionName: "source_trace_7" },
    { x: -2.6499999999999995, y: -4.444999999999999, z: 0, connectionName: "source_trace_9" },
    { x: -20.75, y: -13.97, z: 1, connectionName: "source_trace_12" },
    { x: -20.75, y: -11.43, z: 0, connectionName: "source_trace_14" },
    { x: -20.75, y: -8.89, z: 0, connectionName: "source_trace_16" },
    { x: -2.6499999999999986, y: 0.6306896551724146, z: 3, connectionName: "source_trace_18" },
    { x: -20.75, y: -6.3500000000000005, z: 3, connectionName: "source_trace_18" },
    { x: -2.6499999999999986, y: 2.54, z: 0, connectionName: "source_trace_20" },
    { x: -20.75, y: -3.8100000000000005, z: 0, connectionName: "source_trace_20" },
    { x: -2.6499999999999986, y: 4.205517241379312, z: 3, connectionName: "source_trace_22" },
    { x: -20.75, y: -1.2700000000000014, z: 3, connectionName: "source_trace_22" },
    { x: -2.6499999999999986, y: 5.226896551724138, z: 3, connectionName: "source_trace_23" },
    { x: -20.75, y: 1.2699999999999996, z: 3, connectionName: "source_trace_23" },
    { x: -2.6499999999999986, y: 1.2700000000000005, z: 0, connectionName: "source_trace_19" },
    { x: -20.75, y: 6.35, z: 0, connectionName: "source_trace_19" },
    { x: -2.6499999999999986, y: 3.6948275862068964, z: 3, connectionName: "source_trace_21" },
    { x: -20.75, y: 3.8100000000000005, z: 3, connectionName: "source_trace_21" },
    { x: -2.6499999999999986, y: -18.89, z: 1, connectionName: "source_trace_0" },
    { x: -2.6499999999999986, y: -17.78, z: 1, connectionName: "source_trace_12" },
    { x: -2.6499999999999986, y: -16.669999999999998, z: 0, connectionName: "source_trace_14" },
    { x: -2.6499999999999986, y: -13.34, z: 0, connectionName: "source_trace_15" },
    { x: -2.6499999999999986, y: -14.45, z: 0, connectionName: "source_trace_16" },
    { x: -2.6499999999999986, y: -15.559999999999999, z: 0, connectionName: "source_trace_17" },
    { x: -2.6499999999999986, y: -10.009999999999998, z: 0, connectionName: "source_trace_5" },
    { x: -2.6499999999999986, y: -11.12, z: 0, connectionName: "source_trace_7" },
    { x: -2.6499999999999986, y: -12.23, z: 0, connectionName: "source_trace_9" },
  ],
  center: { x: -11.7, y: 0 },
  width: 18.1,
  height: 40,
}

export default () => {
  const totalConnections = new Set(
    nodeWithPortPoints.portPoints.map((p) => p.connectionName),
  ).size
  const [maxSteps, setMaxSteps] = useState(totalConnections)

  const solver = useMemo(() => {
    const s = new GreedyDescentCrossingViasSolver({ nodeWithPortPoints })
    for (let i = 0; i < maxSteps && !s.solved && !s.failed; i++) {
      s.step()
    }
    return s
  }, [maxSteps])

  const totalVias = solver.solvedRoutes.reduce(
    (sum, r) => sum + r.vias.length,
    0,
  )

  return (
    <div>
      <div className="border p-2 m-2 text-center font-bold">
        Greedy Descent with Crossing Vias — highdensity89
      </div>
      <div className="border p-2 m-2 flex items-center gap-4">
        <label>
          Step: {Math.min(maxSteps, solver.solvedRoutes.length)} /{" "}
          {totalConnections}
        </label>
        <input
          type="range"
          min={0}
          max={totalConnections}
          value={maxSteps}
          onChange={(e) => setMaxSteps(Number(e.target.value))}
          style={{ flex: 1 }}
        />
        <span className="font-bold">
          {solver.solvedRoutes.length} routes | {totalVias} vias
        </span>
        <span>{solver.solved ? "Solved" : solver.error ?? "..."}</span>
      </div>
      {solver.error && (
        <div className="border p-2 m-2 text-red-500">
          Error: {solver.error}
        </div>
      )}
      <InteractiveGraphics graphics={solver.visualize()} />
    </div>
  )
}
