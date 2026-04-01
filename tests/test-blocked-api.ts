import { cdtTriangulate, buildMeshFromRegions, mergeMesh, SearchInstance, rectToPolygon } from "polyanya"

const pad = rectToPolygon(0, 0, 1, 1, 0.2)     // obstacle 0
const wall = rectToPolygon(0, 3, 8, 0.5, 0.2)   // obstacle 1

const result = cdtTriangulate({
  bounds: { minX: -5, maxX: 5, minY: -5, maxY: 5 },
  obstacles: [pad, wall],
})

const rawMesh = buildMeshFromRegions(result)
const mesh = mergeMesh(rawMesh)

console.log(`Obstacle indices: ${mesh.getObstacleIndices()}`)
console.log(`Blocked: ${mesh.polygons.filter(p => p.blocked).length}/${mesh.polygons.length}`)

// Path from inside pad (0,0) to (3,0) — should fail (pad is blocked)
const si1 = new SearchInstance(mesh)
si1.setStartGoal({ x: 0, y: 0 }, { x: 3, y: 0 })
console.log(`\nPad blocked:   ${si1.search() ? `cost=${si1.getCost().toFixed(2)}` : 'BLOCKED'}`)

// Unblock pad (obstacle 0) — path should succeed
mesh.setObstacleBlocked(0, false)
const si2 = new SearchInstance(mesh)
si2.setStartGoal({ x: 0, y: 0 }, { x: 3, y: 0 })
console.log(`Pad unblocked: ${si2.search() ? `cost=${si2.getCost().toFixed(2)}` : 'BLOCKED'}`)

// Wall (obstacle 1) stays blocked — path from (-3,4) to (3,4) should fail
const si3 = new SearchInstance(mesh)
si3.setStartGoal({ x: -3, y: 4 }, { x: 3, y: 4 })
console.log(`Through wall:  ${si3.search() ? `cost=${si3.getCost().toFixed(2)}` : 'BLOCKED'}`)

// Unblock wall too
mesh.setObstacleBlocked(1, false)
const si4 = new SearchInstance(mesh)
si4.setStartGoal({ x: -3, y: 4 }, { x: 3, y: 4 })
console.log(`Wall unblocked:${si4.search() ? `cost=${si4.getCost().toFixed(2)}` : 'BLOCKED'}`)

// Re-block everything
mesh.setObstacleBlocked(0, true)
mesh.setObstacleBlocked(1, true)
const si5 = new SearchInstance(mesh)
si5.setStartGoal({ x: -3, y: 0 }, { x: 3, y: 0 })
console.log(`All re-blocked:${si5.search() ? `cost=${si5.getCost().toFixed(2)}` : 'BLOCKED'}`)
