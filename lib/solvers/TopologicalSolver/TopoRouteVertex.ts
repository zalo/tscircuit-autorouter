import type { Point } from "polyanya"

/**
 * A routing vertex in the topological router.
 * Vertices sit ON CDT edges and form the route path via parent/child links.
 * This mirrors gEDA's toporouter_vertex_t with routingedge/parent/child.
 */
export interface RouteVertex {
  x: number
  y: number
  /** Which CDT edge this vertex sits on (-1 for CDT/terminal vertices) */
  edgeIdx: number
  /** Parameter t ∈ [0,1] along the edge (0 = edge.v0, 1 = edge.v1) */
  t: number
  /** Is this a temporary candidate (true) or committed route vertex (false)? */
  isTemp: boolean
  /** Parent in the A* search tree (toward start) */
  parent: RouteVertex | null
  /** Child in the A* search tree (toward end) */
  child: RouteVertex | null
  /** A* costs */
  gcost: number
  hcost: number
  /** Which route this vertex belongs to (after commit) */
  routeName: string
  /** The net thickness at this point */
  thickness: number
}

/**
 * Create a temporary route vertex on a CDT edge at position t.
 */
export function createTempVertex(
  edgeIdx: number,
  t: number,
  ep0: Point,
  ep1: Point,
  thickness: number,
): RouteVertex {
  return {
    x: ep0.x + t * (ep1.x - ep0.x),
    y: ep0.y + t * (ep1.y - ep0.y),
    edgeIdx,
    t,
    isTemp: true,
    parent: null,
    child: null,
    gcost: Infinity,
    hcost: Infinity,
    routeName: "",
    thickness,
  }
}

/**
 * Create a route vertex at a fixed CDT vertex position (terminal/pad).
 */
export function createFixedVertex(
  x: number,
  y: number,
  thickness: number,
): RouteVertex {
  return {
    x,
    y,
    edgeIdx: -1,
    t: -1,
    isTemp: false,
    parent: null,
    child: null,
    gcost: Infinity,
    hcost: Infinity,
    routeName: "",
    thickness,
  }
}

/**
 * Minimum spacing between two route vertices.
 * = half-thickness of v1 + half-thickness of v2 + clearance
 */
export function minSpacing(
  v1: { thickness: number },
  v2: { thickness: number },
  clearance: number,
): number {
  return v1.thickness / 2 + v2.thickness / 2 + clearance
}
