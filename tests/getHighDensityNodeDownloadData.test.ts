import { expect, test } from "bun:test"
import { getHighDensityNodeDownloadData } from "lib/testing/utils/getHighDensityNodeDownloadData"

test("getHighDensityNodeDownloadData finds pipeline 4 node data from solver outputs", () => {
  const solver = {
    nodeSolver: {
      getOutput: () => ({
        meshNodes: [
          {
            capacityMeshNodeId: "cn_1",
            center: { x: 1, y: 2 },
          },
        ],
      }),
    },
    portPointPathingSolver: {
      getOutput: () => ({
        nodesWithPortPoints: [
          {
            capacityMeshNodeId: "cn_1",
            portPoints: [{ portPointId: "pp1", x: 1, y: 1, z: 0 }],
          },
        ],
        inputNodeWithPortPoints: [
          {
            capacityMeshNodeId: "cn_1",
            portPoints: [{ portPointId: "pp1", connectionNodeIds: [] }],
          },
        ],
      }),
    },
    uniformPortDistributionSolver: {
      getOutput: () => [
        {
          capacityMeshNodeId: "cn_1",
          portPoints: [{ portPointId: "pp1", x: 3, y: 4, z: 0 }],
        },
      ],
    },
  } as any

  expect(getHighDensityNodeDownloadData(solver, "cn_1") as any).toEqual({
    nodeId: "cn_1",
    capacityMeshNode: {
      capacityMeshNodeId: "cn_1",
      center: { x: 1, y: 2 },
    },
    nodeWithPortPoints: {
      capacityMeshNodeId: "cn_1",
      portPoints: [{ portPointId: "pp1", x: 3, y: 4, z: 0 }],
    },
    inputNodeWithPortPoints: {
      capacityMeshNodeId: "cn_1",
      portPoints: [{ portPointId: "pp1", connectionNodeIds: [] }],
    },
  })
})

test("getHighDensityNodeDownloadData falls back to legacy solver collections", () => {
  const solver = {
    nodeTargetMerger: {
      newNodes: [{ capacityMeshNodeId: "cn_2", source: "node-target-merger" }],
    },
    portPointPathingSolver: {
      getNodesWithPortPoints: () => [
        { capacityMeshNodeId: "cn_2", source: "port-point-pathing" },
      ],
    },
  } as any

  expect(getHighDensityNodeDownloadData(solver, "cn_2") as any).toEqual({
    nodeId: "cn_2",
    capacityMeshNode: {
      capacityMeshNodeId: "cn_2",
      source: "node-target-merger",
    },
    nodeWithPortPoints: {
      capacityMeshNodeId: "cn_2",
      source: "port-point-pathing",
    },
    inputNodeWithPortPoints: null,
  })
})
