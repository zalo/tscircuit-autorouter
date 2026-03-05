import type { Point } from "polyanya"

/**
 * A topological vertex in the CDT — either an obstacle corner, a pad center,
 * or a Steiner point inserted during triangulation.
 */
export interface TopoVertex {
  id: number
  x: number
  y: number
  /** Which obstacle this vertex belongs to, or -1 for free-space Steiner points */
  obstacleIndex: number
  /** Which net(s) this vertex is connected to (for pad/pin vertices) */
  connectedNets: string[]
}

/**
 * A CDT triangle in the topological mesh.
 * Triangles are the "regions" that paths traverse.
 */
export interface TopoTriangle {
  id: number
  /** Three vertex indices (CCW order) */
  vertices: [number, number, number]
  /** Adjacent triangle indices (-1 for boundary) */
  neighbors: [number, number, number]
  /** Whether this triangle is inside an obstacle (non-traversable) */
  isObstacle: boolean
}

/**
 * A shared edge between two adjacent triangles.
 * Paths cross from one triangle to another through shared edges.
 * The "candidate points" along this edge are where traces can pass.
 */
export interface TopoEdge {
  /** The two vertex indices forming this edge */
  vertices: [number, number]
  /** The two adjacent triangle indices (-1 for boundary) */
  triangles: [number, number]
  /** Whether this edge is a constraint (obstacle boundary) */
  isConstraint: boolean
  /** Ordered list of path crossings on this edge (for topological ordering) */
  crossings: TopoCrossing[]
}

/**
 * A crossing point where a routed path passes through a CDT edge.
 * The order of crossings along an edge defines the topological embedding.
 */
export interface TopoCrossing {
  /** Which net/connection this crossing belongs to */
  connectionName: string
  /** Parameter t ∈ [0,1] along the edge (0 = vertex[0], 1 = vertex[1]) */
  t: number
  /** The actual point */
  point: Point
}

/**
 * A routed path through the CDT, stored as a sequence of triangle crossings.
 * This is the topological representation — the actual geometry comes from
 * rubber-banding.
 */
export interface TopoRoute {
  connectionName: string
  /** Sequence of CDT edges crossed (defines the topology) */
  edgeCrossings: Array<{
    edgeIndex: number
    /** Parameter t along the edge */
    t: number
  }>
  /** The rubber-banded geometric path */
  path: Point[]
  /** Start/end points (pad centers) */
  start: Point
  end: Point
  /** Layer assignments */
  startLayerZ: number
  endLayerZ: number
  routeLayerZ: number
}

/**
 * The complete topological mesh — CDT with routing annotations.
 */
export interface TopoMesh {
  vertices: TopoVertex[]
  triangles: TopoTriangle[]
  edges: TopoEdge[]
  /** Map from vertex pair key to edge index */
  edgeMap: Map<string, number>
}

export interface ResolvedPath {
  connectionName: string
  route: Array<{ x: number; y: number; z: number }>
  vias: Array<{ x: number; y: number }>
}
