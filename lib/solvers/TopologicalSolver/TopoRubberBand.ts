/**
 * Rubber-band optimization — direct port of gEDA toporouter.c
 * oproute_rubberband_segment() and supporting functions.
 *
 * Converts a topological path (sequence of vertices on CDT edges)
 * into a smooth geometric path by inserting arcs around obstacle vertices.
 */

import type { Point } from "polyanya"
import type { RawCdt } from "./TopologicalCdt"
import type { RouteVertex } from "./TopoRouteVertex"

/** An arc that wraps around an obstacle vertex at clearance distance */
export interface TopoArc {
  /** The obstacle vertex this arc wraps around */
  centre: Point
  /** Arc radius (= clearance from centre) */
  r: number
  /** Winding direction: +1 = CCW, -1 = CW */
  dir: number
  /** Entry tangent point (coming from previous segment/arc) */
  x0: number
  y0: number
  /** Exit tangent point (going to next segment/arc) */
  x1: number
  y1: number
}

/** Terminal type: either a fixed point or an arc */
type Terminal = { kind: "point"; x: number; y: number } | { kind: "arc"; arc: TopoArc }

function termEntryXY(t: Terminal): [number, number] {
  return t.kind === "point" ? [t.x, t.y] : [t.arc.x0, t.arc.y0]
}

function termExitXY(t: Terminal): [number, number] {
  return t.kind === "point" ? [t.x, t.y] : [t.arc.x1, t.arc.y1]
}

/** Winding: +1 left, -1 right, 0 collinear */
function wind(ax: number, ay: number, bx: number, by: number, cx: number, cy: number): number {
  const cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
  if (cross > 1e-9) return 1
  if (cross < -1e-9) return -1
  return 0
}

/** Perpendicular foot from point (px,py) onto line (x0,y0)→(x1,y1). Returns [fx,fy,t] */
function perpFoot(px: number, py: number, x0: number, y0: number, x1: number, y1: number): [number, number, number] {
  const dx = x1 - x0, dy = y1 - y0
  const len2 = dx * dx + dy * dy
  if (len2 < 1e-18) return [x0, y0, 0]
  const t = ((px - x0) * dx + (py - y0) * dy) / len2
  return [x0 + t * dx, y0 + t * dy, t]
}

function dist(x0: number, y0: number, x1: number, y1: number): number {
  return Math.hypot(x1 - x0, y1 - y0)
}

/**
 * Check if an obstacle vertex's clearance circle INTERSECTS the line segment.
 * The edge is "cutting through" the segment (endpoints on opposite sides).
 * Returns violation depth (>0 if clearance violated), or -1 if no violation.
 *
 * Port of gEDA check_intersect_vertex().
 */
function checkIntersectVertex(
  x0: number, y0: number, x1: number, y1: number,
  arcv: Point, ms: number,
): { d: number; arcWind: number } | null {
  const [fx, fy, t] = perpFoot(arcv.x, arcv.y, x0, y0, x1, y1)
  if (t < 0 || t > 1) return null // foot outside segment

  const d = dist(arcv.x, arcv.y, fx, fy)
  if (d > ms - 1e-6) return null // no violation

  const arcWind = wind(x0, y0, arcv.x, arcv.y, x1, y1)
  return { d: ms - d, arcWind }
}

/**
 * Check if an obstacle vertex needs to push the segment away even though
 * it doesn't directly intersect. The vertex is on one side of the segment.
 * Returns violation depth, or -1 if no violation.
 *
 * Port of gEDA check_non_intersect_vertex().
 */
function checkNonIntersectVertex(
  x0: number, y0: number, x1: number, y1: number,
  arcv: Point, opv: Point, edgeWind: number, ms: number,
): { d: number; arcWind: number } | null {
  const [fx, fy, t] = perpFoot(arcv.x, arcv.y, x0, y0, x1, y1)

  // Use the closer segment endpoint if the foot is outside the segment
  let lx: number, ly: number
  if (t < 0 || t > 1) {
    if (dist(x0, y0, arcv.x, arcv.y) < dist(x1, y1, arcv.x, arcv.y)) {
      lx = x0; ly = y0
    } else {
      lx = x1; ly = y1
    }
  } else {
    lx = fx; ly = fy
  }

  const d = dist(arcv.x, arcv.y, lx, ly)

  // Check if arcv and opv are on the same side of the perpendicular through arcv
  // If so, the arc would point away from the segment — no violation
  const perpDx = -(ly - arcv.y), perpDy = lx - arcv.x
  const w1 = wind(arcv.x, arcv.y, arcv.x + perpDx, arcv.y + perpDy, lx, ly)
  const w2 = wind(arcv.x, arcv.y, arcv.x + perpDx, arcv.y + perpDy, opv.x, opv.y)
  if (!w2 || w1 === w2) return null

  const arcWind = wind(x0, y0, arcv.x, arcv.y, x1, y1)
  return { d: d + ms, arcWind }
}

/**
 * Calculate tangent point from a terminal point to an arc.
 * Port of gEDA calculate_term_to_arc().
 *
 * Sets arc.x0/y0 (if dir=0, entry) or arc.x1/y1 (if dir=1, exit).
 */
function calculateTermToArc(vx: number, vy: number, arc: TopoArc, dir: number) {
  const d = dist(vx, vy, arc.centre.x, arc.centre.y)
  if (d < arc.r + 1e-9) {
    // Point is inside or on the arc circle — degenerate case
    if (dir === 0) { arc.x0 = vx; arc.y0 = vy }
    else { arc.x1 = vx; arc.y1 = vy }
    return
  }

  const theta = Math.acos(Math.min(1, arc.r / d))
  const a = arc.r * Math.sin(theta) // perpendicular offset
  const b = arc.r * Math.cos(theta) // parallel offset

  // Point on line from arc centre toward v, at distance b
  const dx = vx - arc.centre.x, dy = vy - arc.centre.y
  const dLen = Math.hypot(dx, dy)
  const bx = arc.centre.x + (dx / dLen) * b
  const by = arc.centre.y + (dy / dLen) * b

  // Two candidate tangent points, perpendicular to centre→v line at (bx,by)
  const perpX = -dy / dLen, perpY = dx / dLen
  const a0x = bx + perpX * a, a0y = by + perpY * a
  const a1x = bx - perpX * a, a1y = by - perpY * a

  let winddir = wind(vx, vy, a0x, a0y, arc.centre.x, arc.centre.y)
  if (!winddir) {
    // Degenerate
    if (dir === 0) { arc.x0 = vx; arc.y0 = vy }
    else { arc.x1 = vx; arc.y1 = vy }
    return
  }

  if (dir) winddir = -winddir

  if (winddir === arc.dir) {
    if (!dir) { arc.x0 = a0x; arc.y0 = a0y }
    else { arc.x1 = a0x; arc.y1 = a0y }
  } else {
    if (!dir) { arc.x0 = a1x; arc.y0 = a1y }
    else { arc.x1 = a1x; arc.y1 = a1y }
  }
}

/**
 * Calculate tangent points between two arcs.
 * Port of gEDA calculate_arc_to_arc().
 * Sets parc.x1/y1 and arc.x0/y0.
 */
function calculateArcToArc(parc: TopoArc, arc: TopoArc): boolean {
  const d = dist(parc.centre.x, parc.centre.y, arc.centre.x, arc.centre.y)
  if (d < 1e-9) return true // degenerate

  const bigr = parc.r > arc.r ? parc : arc
  const smallr = parc.r > arc.r ? arc : parc

  if (parc.dir === arc.dir) {
    // Same direction — external tangent
    const ratio = (bigr.r - smallr.r) / d
    if (Math.abs(ratio) > 1) return true
    const theta = Math.acos(ratio)
    const a = bigr.r * Math.sin(theta)
    const b = bigr.r * Math.cos(theta)

    const dx = smallr.centre.x - bigr.centre.x
    const dy = smallr.centre.y - bigr.centre.y
    const dLen = Math.hypot(dx, dy)
    const bx = bigr.centre.x + (dx / dLen) * b
    const by = bigr.centre.y + (dy / dLen) * b

    const perpX = -dy / dLen, perpY = dx / dLen
    const a0x = bx + perpX * a, a0y = by + perpY * a
    const a1x = bx - perpX * a, a1y = by - perpY * a

    let winddir = wind(smallr.centre.x, smallr.centre.y, a0x, a0y, bigr.centre.x, bigr.centre.y)
    if (!winddir) return true

    if (bigr === parc) winddir = -winddir

    if (winddir === bigr.dir) {
      if (bigr === arc) { bigr.x0 = a0x; bigr.y0 = a0y }
      else { bigr.x1 = a0x; bigr.y1 = a0y }
    } else {
      if (bigr === arc) { bigr.x0 = a1x; bigr.y0 = a1y }
      else { bigr.x1 = a1x; bigr.y1 = a1y }
    }

    // Small arc tangent
    const sa = smallr.r * Math.sin(theta)
    const sb = smallr.r * Math.cos(theta)
    const sbx = smallr.centre.x + (-dx / dLen) * (-sb)
    const sby = smallr.centre.y + (-dy / dLen) * (-sb)
    const sa0x = sbx + perpX * sa, sa0y = sby + perpY * sa
    const sa1x = sbx - perpX * sa, sa1y = sby - perpY * sa

    if (winddir === bigr.dir) {
      if (bigr === arc) { smallr.x1 = sa0x; smallr.y1 = sa0y }
      else { smallr.x0 = sa0x; smallr.y0 = sa0y }
    } else {
      if (bigr === arc) { smallr.x1 = sa1x; smallr.y1 = sa1y }
      else { smallr.x0 = sa1x; smallr.y0 = sa1y }
    }
  } else {
    // Opposite direction — cross tangent
    const ratio = (bigr.r + smallr.r) / d
    if (ratio > 1) return true
    const theta = Math.acos(ratio)
    const a = bigr.r * Math.sin(theta)
    const b = bigr.r * Math.cos(theta)

    const dx = smallr.centre.x - bigr.centre.x
    const dy = smallr.centre.y - bigr.centre.y
    const dLen = Math.hypot(dx, dy)
    const bx = bigr.centre.x + (dx / dLen) * b
    const by = bigr.centre.y + (dy / dLen) * b

    const perpX = -dy / dLen, perpY = dx / dLen
    const a0x = bx + perpX * a, a0y = by + perpY * a

    if (bigr === arc) { bigr.x0 = a0x; bigr.y0 = a0y }
    else { bigr.x1 = a0x; bigr.y1 = a0y }

    const sa = smallr.r * Math.sin(theta)
    const sb = smallr.r * Math.cos(theta)
    const sbx = smallr.centre.x + (-dx / dLen) * sb
    const sby = smallr.centre.y + (-dy / dLen) * sb
    const sa0x = sbx + perpX * sa, sa0y = sby + perpY * sa

    if (bigr === arc) { smallr.x1 = sa0x; smallr.y1 = sa0y }
    else { smallr.x0 = sa0x; smallr.y0 = sa0y }
  }

  return false
}

interface RubberbandCandidate {
  arcv: Point    // obstacle vertex to arc around
  r: number      // arc radius
  d: number      // violation depth (higher = worse)
  arcWind: number // winding direction
  pathIdx: number // index in path where violation occurs
}

/**
 * Core rubber-band algorithm — port of gEDA oproute_rubberband_segment().
 *
 * Given a line segment from t1 to t2, walk the path vertices between them.
 * For each vertex on a CDT edge, check both edge endpoints (obstacle vertices)
 * for clearance violations against the segment. Find the worst violation,
 * create an arc around it, then recurse on the two sub-segments.
 *
 * Returns a list of arcs (in order from t1 to t2).
 */
export function rubberbandSegment(
  cdt: RawCdt,
  path: RouteVertex[],
  pathStart: number,
  pathEnd: number,
  t1: Terminal,
  t2: Terminal,
  margin: number,
  traceWidth: number,
): TopoArc[] {
  const [x0, y0] = termExitXY(t1)
  const [x1, y1] = termEntryXY(t2)

  if (pathEnd - pathStart < 1) return []
  if (dist(x0, y0, x1, y1) < 1e-9) return []

  const ms = traceWidth / 2 + margin
  const candidates: RubberbandCandidate[] = []

  // Walk path vertices between t1 and t2
  for (let pi = pathStart; pi < pathEnd; pi++) {
    const v = path[pi]!
    if (v.edgeIdx < 0) continue

    const edge = cdt.edges[v.edgeIdx]!
    const ev0 = cdt.pts[edge.v0]!
    const ev1 = cdt.pts[edge.v1]!

    const v0wind = wind(x0, y0, x1, y1, ev0.x, ev0.y)
    const v1wind = wind(x0, y0, x1, y1, ev1.x, ev1.y)

    if (!v0wind && !v1wind) continue // edge collinear with segment

    if (v0wind && v1wind && v0wind !== v1wind) {
      // Edge cuts through segment — check both endpoints for intersection
      if (cdt.obstacleVertices.has(edge.v0)) {
        const result = checkIntersectVertex(x0, y0, x1, y1, ev0, ms)
        if (result && result.d > 1e-6) {
          candidates.push({ arcv: ev0, r: ms, d: result.d, arcWind: result.arcWind, pathIdx: pi })
        }
      }
      if (cdt.obstacleVertices.has(edge.v1)) {
        const result = checkIntersectVertex(x0, y0, x1, y1, ev1, ms)
        if (result && result.d > 1e-6) {
          candidates.push({ arcv: ev1, r: ms, d: result.d, arcWind: result.arcWind, pathIdx: pi })
        }
      }
    } else {
      // Edge on one side — check for non-intersecting violations
      if (cdt.obstacleVertices.has(edge.v0)) {
        const result = checkNonIntersectVertex(x0, y0, x1, y1, ev0, ev1, v0wind, ms)
        if (result && result.d > 1e-6) {
          candidates.push({ arcv: ev0, r: ms, d: result.d, arcWind: result.arcWind, pathIdx: pi })
        }
      }
      if (cdt.obstacleVertices.has(edge.v1)) {
        const result = checkNonIntersectVertex(x0, y0, x1, y1, ev1, ev0, v1wind, ms)
        if (result && result.d > 1e-6) {
          candidates.push({ arcv: ev1, r: ms, d: result.d, arcWind: result.arcWind, pathIdx: pi })
        }
      }
    }
  }

  if (candidates.length === 0) return []

  // Sort by violation depth, take worst
  candidates.sort((a, b) => b.d - a.d)
  const best = candidates[0]!

  // Create new arc around the worst-violating obstacle vertex
  const newArc: TopoArc = {
    centre: best.arcv,
    r: best.r,
    dir: best.arcWind || 1,
    x0: 0, y0: 0, x1: 0, y1: 0,
  }

  // Calculate tangent points
  if (t1.kind === "point") {
    calculateTermToArc(t1.x, t1.y, newArc, 0)
  } else {
    if (calculateArcToArc(t1.arc, newArc)) return [] // degenerate
  }

  if (t2.kind === "point") {
    calculateTermToArc(t2.x, t2.y, newArc, 1)
  } else {
    if (calculateArcToArc(newArc, t2.arc)) return [] // degenerate
  }

  // Recurse on sub-segments
  const leftArcs = rubberbandSegment(
    cdt, path, pathStart, best.pathIdx,
    t1, { kind: "arc", arc: newArc },
    margin, traceWidth,
  )
  const rightArcs = rubberbandSegment(
    cdt, path, best.pathIdx + 1, pathEnd,
    { kind: "arc", arc: newArc }, t2,
    margin, traceWidth,
  )

  return [...leftArcs, newArc, ...rightArcs]
}

/**
 * Convert a topological path + arcs into a geometric point path.
 * Arcs are approximated with line segments.
 */
export function arcsToPath(
  start: Point,
  end: Point,
  arcs: TopoArc[],
  arcSegments: number = 6,
): Point[] {
  if (arcs.length === 0) return [start, end]

  const result: Point[] = [start]

  for (let i = 0; i < arcs.length; i++) {
    const arc = arcs[i]!

    // Entry tangent point
    result.push({ x: arc.x0, y: arc.y0 })

    // Arc approximation
    const startAngle = Math.atan2(arc.y0 - arc.centre.y, arc.x0 - arc.centre.x)
    const endAngle = Math.atan2(arc.y1 - arc.centre.y, arc.x1 - arc.centre.x)

    let sweep = endAngle - startAngle
    if (arc.dir > 0) {
      // CCW
      while (sweep < 0) sweep += Math.PI * 2
    } else {
      // CW
      while (sweep > 0) sweep -= Math.PI * 2
    }

    const steps = Math.max(2, Math.round(Math.abs(sweep) / (Math.PI / arcSegments)))
    for (let s = 1; s < steps; s++) {
      const a = startAngle + (sweep * s) / steps
      result.push({
        x: arc.centre.x + Math.cos(a) * arc.r,
        y: arc.centre.y + Math.sin(a) * arc.r,
      })
    }

    // Exit tangent point
    result.push({ x: arc.x1, y: arc.y1 })
  }

  result.push(end)
  return result
}
