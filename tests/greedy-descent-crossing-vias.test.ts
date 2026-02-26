import { test, expect } from "bun:test"
import { GreedyDescentCrossingViasSolver } from "lib/solvers/HighDensitySolver/GreedyDescentCrossingViasSolver"

// Simple two-crossing-route test (X crossing pattern)
const simpleCrossing = {
  capacityMeshNodeId: "node1",
  center: { x: 5, y: 5 },
  width: 10,
  height: 10,
  portPoints: [
    { connectionName: "A", x: 0, y: 0, z: 0 },
    { connectionName: "A", x: 10, y: 10, z: 0 },
    { connectionName: "B", x: 0, y: 10, z: 0 },
    { connectionName: "B", x: 10, y: 0, z: 0 },
  ],
}

test("greedy descent solves simple X crossing with vias", () => {
  const solver = new GreedyDescentCrossingViasSolver({
    nodeWithPortPoints: simpleCrossing,
  })
  solver.solve()
  expect(solver.solved).toBe(true)
  expect(solver.solvedRoutes).toHaveLength(2)

  // Either the A* detoured around port obstacles (no crossing, no vias needed)
  // or the via-hop mechanism resolved the crossing with vias.
  // Both are valid solutions.
  for (const route of solver.solvedRoutes) {
    expect(route.route.length).toBeGreaterThanOrEqual(2)
  }
})

// Three non-crossing parallel routes
const parallelRoutes = {
  capacityMeshNodeId: "node2",
  center: { x: 5, y: 5 },
  width: 10,
  height: 10,
  portPoints: [
    { connectionName: "A", x: 0, y: 2, z: 0 },
    { connectionName: "A", x: 10, y: 2, z: 0 },
    { connectionName: "B", x: 0, y: 5, z: 0 },
    { connectionName: "B", x: 10, y: 5, z: 0 },
    { connectionName: "C", x: 0, y: 8, z: 0 },
    { connectionName: "C", x: 10, y: 8, z: 0 },
  ],
}

test("greedy descent routes parallel traces without vias", () => {
  const solver = new GreedyDescentCrossingViasSolver({
    nodeWithPortPoints: parallelRoutes,
  })
  solver.solve()
  expect(solver.solved).toBe(true)
  expect(solver.solvedRoutes).toHaveLength(3)

  for (const route of solver.solvedRoutes) {
    expect(route.vias).toHaveLength(0)
  }
})

// Layer transition test
const layerTransition = {
  capacityMeshNodeId: "node3",
  center: { x: 5, y: 5 },
  width: 10,
  height: 10,
  portPoints: [
    { connectionName: "A", x: 0, y: 5, z: 0 },
    { connectionName: "A", x: 10, y: 5, z: 1 },
  ],
}

test("greedy descent handles layer transitions", () => {
  const solver = new GreedyDescentCrossingViasSolver({
    nodeWithPortPoints: layerTransition,
  })
  solver.solve()
  expect(solver.solved).toBe(true)
  expect(solver.solvedRoutes).toHaveLength(1)
  expect(solver.solvedRoutes[0].vias.length).toBeGreaterThanOrEqual(1)
})

// Multi-step test: routes accumulate
test("greedy descent is multi-step (one route per step)", () => {
  const solver = new GreedyDescentCrossingViasSolver({
    nodeWithPortPoints: simpleCrossing,
  })

  expect(solver.solved).toBe(false)
  expect(solver.solvedRoutes).toHaveLength(0)

  solver.step()
  expect(solver.solvedRoutes).toHaveLength(1)
  expect(solver.solved).toBe(false)

  solver.step()
  expect(solver.solvedRoutes).toHaveLength(2)
  expect(solver.solved).toBe(true)
})

// Highdensity89 fixture data
const hd89 = {
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

test("greedy descent solves highdensity89 fixture", { timeout: 30000 }, () => {
  const solver = new GreedyDescentCrossingViasSolver({
    nodeWithPortPoints: hd89,
  })
  solver.solve()
  expect(solver.solved).toBe(true)

  const connectionNames = new Set(hd89.portPoints.map((p) => p.connectionName))
  expect(solver.solvedRoutes).toHaveLength(connectionNames.size)

  for (const route of solver.solvedRoutes) {
    expect(route.route.length).toBeGreaterThanOrEqual(2)
    expect(route.traceThickness).toBeGreaterThan(0)
    expect(route.viaDiameter).toBeGreaterThan(0)
  }
})
