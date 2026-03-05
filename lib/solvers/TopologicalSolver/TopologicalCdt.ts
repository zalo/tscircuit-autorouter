import cdt2d from "cdt2d"
import { resolveConstraintCrossings } from "polyanya"
import { rectToPolygon } from "polyanya"
import type { Point } from "polyanya"
import type { SimpleRouteJson } from "../../types"

/**
 * Raw CDT triangle — stores vertex indices and neighbor triangle indices.
 */
export interface CdtTriangle {
  /** Three vertex indices (CCW winding) */
  v: [number, number, number]
  /** Adjacent triangle index across each edge (-1 = boundary/obstacle).
   *  n[0] = neighbor across edge v[1]→v[2]
   *  n[1] = neighbor across edge v[2]→v[0]
   *  n[2] = neighbor across edge v[0]→v[1] */
  n: [number, number, number]
  /** Is this triangle inside an obstacle (non-traversable)? */
  obstacle: boolean
}

/**
 * A CDT edge shared by two triangles. Routes pass through these edges.
 * The gEDA toporouter stores an ordered list of route crossings per edge.
 */
export interface CdtEdge {
  /** The two vertex indices forming this edge */
  v0: number
  v1: number
  /** Adjacent triangle indices (-1 = boundary) */
  t0: number
  t1: number
  /** Is this a constraint edge (obstacle boundary)? */
  isConstraint: boolean
  /** Length of this edge */
  length: number
  /** Ordered route crossings on this edge (sorted by t parameter, 0=v0, 1=v1) */
  crossings: EdgeCrossing[]
}

/**
 * A route crossing on a CDT edge.
 */
export interface EdgeCrossing {
  connectionName: string
  /** Parameter t ∈ [0,1] along the edge */
  t: number
  /** The actual crossing point */
  point: Point
}

/**
 * The complete raw CDT with topology — triangles, edges, and adjacency.
 */
export interface RawCdt {
  /** Vertex positions */
  pts: Point[]
  /** Triangles with adjacency */
  triangles: CdtTriangle[]
  /** Edges with crossing data */
  edges: CdtEdge[]
  /** Map from sorted vertex pair "v0,v1" to edge index */
  edgeMap: Map<string, number>
  /** Which vertex indices are obstacle-related (constraint ring vertices) */
  obstacleVertices: Set<number>
  /** Map from obstacle index to its constraint vertex indices */
  obstacleRings: number[][]
}

function edgeKey(a: number, b: number): string {
  return a < b ? `${a},${b}` : `${b},${a}`
}

function pointInPolygon(px: number, py: number, poly: Point[]): boolean {
  let inside = false
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i]!.x,
      yi = poly[i]!.y
    const xj = poly[j]!.x,
      yj = poly[j]!.y
    if (yi > py !== yj > py && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) {
      inside = !inside
    }
  }
  return inside
}

/**
 * Build a raw CDT from SRJ obstacles, preserving full triangle/edge topology.
 * This is the foundation for the topological router — routes traverse through
 * CDT edges, and the edge crossing order defines the routing topology.
 */
export function buildRawCdt(
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
  obstaclePolygons: Point[][],
): RawCdt | null {
  const pts: [number, number][] = []
  const constraintEdges: [number, number][] = []
  const { minX, maxX, minY, maxY } = bounds

  // --- Bounds ring ---
  const edgeSamples = 10
  const boundsStart = pts.length
  pts.push([minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY])
  for (let i = 1; i < edgeSamples; i++) {
    const t = i / edgeSamples
    pts.push([minX + t * (maxX - minX), minY])
    pts.push([maxX, minY + t * (maxY - minY)])
    pts.push([maxX - t * (maxX - minX), maxY])
    pts.push([minX, maxY - t * (maxY - minY)])
  }
  const boundsEnd = pts.length
  // Build bounds constraint ring
  const boundsEdgePoints: number[][] = [[], [], [], []]
  boundsEdgePoints[0]!.push(boundsStart)
  boundsEdgePoints[1]!.push(boundsStart + 1)
  boundsEdgePoints[2]!.push(boundsStart + 2)
  boundsEdgePoints[3]!.push(boundsStart + 3)
  for (let i = 1; i < edgeSamples; i++) {
    const base = boundsStart + 4 + (i - 1) * 4
    boundsEdgePoints[0]!.push(base)
    boundsEdgePoints[1]!.push(base + 1)
    boundsEdgePoints[2]!.push(base + 2)
    boundsEdgePoints[3]!.push(base + 3)
  }
  boundsEdgePoints[0]!.push(boundsStart + 1)
  boundsEdgePoints[1]!.push(boundsStart + 2)
  boundsEdgePoints[2]!.push(boundsStart + 3)
  boundsEdgePoints[3]!.push(boundsStart)
  for (const side of boundsEdgePoints) {
    for (let i = 0; i < side.length - 1; i++) {
      constraintEdges.push([side[i]!, side[i + 1]!])
    }
  }

  // --- Obstacle polygon rings ---
  const obstacleVertexSet = new Set<number>()
  const obstacleRings: number[][] = []
  const constraintEdgeSet = new Set<string>()

  for (const obstacle of obstaclePolygons) {
    if (obstacle.length < 3) continue

    // Deduplicate
    const deduped: Point[] = []
    for (let i = 0; i < obstacle.length; i++) {
      const p = obstacle[i]!
      const prev = deduped.length > 0 ? deduped[deduped.length - 1]! : null
      if (
        !prev ||
        Math.abs(p.x - prev.x) > 1e-9 ||
        Math.abs(p.y - prev.y) > 1e-9
      ) {
        deduped.push(p)
      }
    }
    if (deduped.length > 1) {
      const first = deduped[0]!
      const last = deduped[deduped.length - 1]!
      if (
        Math.abs(first.x - last.x) < 1e-9 &&
        Math.abs(first.y - last.y) < 1e-9
      ) {
        deduped.pop()
      }
    }
    if (deduped.length < 3) continue

    const ringStart = pts.length
    const ring: number[] = []
    for (let i = 0; i < deduped.length; i++) {
      const p = deduped[i]!
      pts.push([p.x + ((i % 7) - 3) * 1e-8, p.y + ((i % 5) - 2) * 1e-8])
      const vi = ringStart + i
      obstacleVertexSet.add(vi)
      ring.push(vi)
    }
    obstacleRings.push(ring)

    for (let i = 0; i < deduped.length; i++) {
      const a = ringStart + i
      const b = ringStart + ((i + 1) % deduped.length)
      constraintEdges.push([a, b])
      constraintEdgeSet.add(edgeKey(a, b))
    }
  }

  // --- Resolve crossings and run CDT ---
  const ringBoundaries: number[] = [] // not used by our resolver path
  const resolved = resolveConstraintCrossings(pts, constraintEdges, ringBoundaries)

  let triangles: [number, number, number][]
  try {
    triangles = cdt2d(resolved.pts, resolved.constraintEdges, {
      exterior: false,
    })
  } catch {
    const jitteredPts = resolved.pts.map((p, i) => {
      if (i < boundsEnd) return p
      return [
        p[0] + (Math.random() - 0.5) * 1e-5,
        p[1] + (Math.random() - 0.5) * 1e-5,
      ] as [number, number]
    })
    try {
      triangles = cdt2d(jitteredPts, resolved.constraintEdges, {
        exterior: false,
      })
      for (let i = 0; i < jitteredPts.length; i++) {
        resolved.pts[i] = jitteredPts[i]!
      }
    } catch {
      return null
    }
  }

  // --- Convert pts to Point[] ---
  const points: Point[] = resolved.pts.map(([x, y]) => ({ x, y }))

  // --- Build constraint edge set from resolved edges ---
  const resolvedConstraintSet = new Set<string>()
  for (const [a, b] of resolved.constraintEdges) {
    resolvedConstraintSet.add(edgeKey(a, b))
  }

  // --- Mark obstacle triangles ---
  // A triangle is inside an obstacle if ALL THREE of its vertices belong
  // to the same obstacle ring. This is more robust than centroid-in-polygon
  // because jitter and resolveConstraintCrossings can shift vertices slightly.
  const vertexToRing = new Map<number, number>()
  for (let ri = 0; ri < obstacleRings.length; ri++) {
    for (const vi of obstacleRings[ri]!) {
      vertexToRing.set(vi, ri)
    }
  }

  const cdtTriangles: CdtTriangle[] = triangles.map((tri) => {
    const [a, b, c] = tri
    const rA = vertexToRing.get(a)
    const rB = vertexToRing.get(b)
    const rC = vertexToRing.get(c)

    // All three vertices from the same obstacle ring → inside obstacle
    let isObstacle = false
    if (rA !== undefined && rA === rB && rA === rC) {
      isObstacle = true
    }

    // Fallback: centroid-in-polygon check for triangles with mixed vertices
    if (!isObstacle) {
      const cx = (points[a]!.x + points[b]!.x + points[c]!.x) / 3
      const cy = (points[a]!.y + points[b]!.y + points[c]!.y) / 3
      isObstacle = obstaclePolygons.some((poly) =>
        pointInPolygon(cx, cy, poly),
      )
    }

    return {
      v: [a, b, c] as [number, number, number],
      n: [-1, -1, -1] as [number, number, number],
      obstacle: isObstacle,
    }
  })

  // --- Build edge map and triangle adjacency ---
  // For each edge, track which triangles share it
  const edgeTris = new Map<string, number[]>()
  for (let ti = 0; ti < cdtTriangles.length; ti++) {
    const tri = cdtTriangles[ti]!
    const edges = [
      [tri.v[1], tri.v[2]], // opposite v[0], stored at n[0]
      [tri.v[2], tri.v[0]], // opposite v[1], stored at n[1]
      [tri.v[0], tri.v[1]], // opposite v[2], stored at n[2]
    ]
    for (let ei = 0; ei < 3; ei++) {
      const key = edgeKey(edges[ei]![0]!, edges[ei]![1]!)
      if (!edgeTris.has(key)) edgeTris.set(key, [])
      edgeTris.get(key)!.push(ti)
    }
  }

  // Set adjacency
  for (let ti = 0; ti < cdtTriangles.length; ti++) {
    const tri = cdtTriangles[ti]!
    const edgesOfTri = [
      [tri.v[1], tri.v[2]],
      [tri.v[2], tri.v[0]],
      [tri.v[0], tri.v[1]],
    ]
    for (let ei = 0; ei < 3; ei++) {
      const key = edgeKey(edgesOfTri[ei]![0]!, edgesOfTri[ei]![1]!)
      const sharing = edgeTris.get(key)!
      for (const otherTi of sharing) {
        if (otherTi !== ti) {
          tri.n[ei] = otherTi
          break
        }
      }
    }
  }

  // --- Build CdtEdge objects ---
  const edgeMapResult = new Map<string, number>()
  const cdtEdges: CdtEdge[] = []

  for (const [key, tris] of edgeTris) {
    const [v0s, v1s] = key.split(",")
    const v0 = Number.parseInt(v0s!)
    const v1 = Number.parseInt(v1s!)
    const t0 = tris[0] ?? -1
    const t1 = tris[1] ?? -1
    const dx = points[v1]!.x - points[v0]!.x
    const dy = points[v1]!.y - points[v0]!.y
    const length = Math.hypot(dx, dy)

    const edgeIdx = cdtEdges.length
    edgeMapResult.set(key, edgeIdx)
    cdtEdges.push({
      v0,
      v1,
      t0,
      t1,
      isConstraint: resolvedConstraintSet.has(key),
      length,
      crossings: [],
    })
  }

  return {
    pts: points,
    triangles: cdtTriangles,
    edges: cdtEdges,
    edgeMap: edgeMapResult,
    obstacleVertices: obstacleVertexSet,
    obstacleRings,
  }
}
