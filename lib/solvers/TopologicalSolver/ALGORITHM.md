# gEDA Toporouter Algorithm (ported to TypeScript)

## Data Model

### Edge Routing List
Each CDT edge has a `routing` list: an ordered list of route vertices
sitting ON that edge, sorted by distance from edge vertex v1.

When `apply_route()` commits a path, each path vertex that has a
`routingedge` gets inserted into that edge's routing list (sorted).

This list IS the topological state. The order of vertices from different
nets on a shared edge defines the crossing topology.

### Vertex Types
- **CDT vertices**: Fixed obstacle corners, pad centers (not TEMP)
- **TEMP vertices**: Route waypoints placed on edges during A* search
- **ROUTE vertices**: Committed route vertices (TEMP promoted after routing)

### Key Principle
Route vertices live ON CDT edges. The path is a linked list of vertices
(parent/child pointers). Each vertex knows which edge it sits on
(`routingedge`). Moving a vertex along its edge automatically changes
the geometric path.

## Phase 1: A* Routing (`route()`)

For each connection:
1. Find closest src/dest vertex pair
2. Initialize A* open list with src vertex
3. Pop lowest f-cost vertex from heap
4. Generate candidate points via `compute_candidate_points()`:

   **If current vertex is a CDT vertex (not TEMP):**
   - Iterate all triangles around this vertex
   - For each triangle, call `triangle_candidate_points_from_vertex()`
   - This generates candidate TEMP vertices on the two opposite edges

   **If current vertex is a TEMP vertex on edge E:**
   - Find which triangle is on the OPPOSITE side of E from parent
     (using `prevwind != vertex_wind(...)` winding check)
   - Call `triangle_candidate_points_from_edge()` for that triangle only
   - This generates candidates on the two edges of the triangle that
     aren't E

5. For each candidate:
   - Compute g-cost (distance from start) and h-cost (heuristic to dest)
   - Insert into open list or update if better path found
6. When dest reached, extract path via parent pointers

### candidate_vertices(v1, v2, dest, edge)
Given a gap between v1 and v2 on an edge:
- Calculate total distance, min spacing from each end
- If flow >= capacity: return nothing (edge full)
- If tight: return midpoint only
- Otherwise: return up to 3 candidates:
  - Near v1 (at min_spacing distance)
  - Near v2 (at min_spacing distance)
  - Center of remaining gap

### Winding Check (CRITICAL for topology)
From a TEMP vertex on edge E:
- `prevwind` = which side of E the parent vertex is on
- Only expand into the triangle on the OPPOSITE side
- This ensures routes don't backtrack and maintains topological
  consistency (different routes around an obstacle go different ways)

## Phase 2: Apply Route (`apply_route()`)

After A* finds a path:
- Walk the path vertex list
- For each vertex with a routingedge:
  - Insert it into that edge's routing list (sorted by distance from v1)
  - Set parent/child links
- The edge routing lists now reflect the new route's presence
- Subsequent routes see these vertices when computing capacity/candidates

## Phase 3: Space Edges (`space_edge()`)

After ALL routes are committed:
- For each non-constraint edge with routing vertices:
- Run 100 iterations of force-based relaxation:
  - For each routing vertex:
    - Calculate force from prev neighbor (or edge v1): push away if too close
    - Calculate force from next neighbor (or edge v2): push away if too close
  - Move each vertex along the edge by force * 0.1 (damping)
  - Stop when equilibrium reached (all forces < EPSILON)

This spreads route vertices evenly along shared edges while respecting
minimum spacing requirements.

## Phase 4: Rubber-Band (`oproute_rubberband_segment()`)

For each routed path:
1. Consider the line segment from terminal t1 to terminal t2
2. Walk along the path vertices between t1 and t2
3. For each path vertex v on routing edge E:
   - Get E's two CDT endpoint vertices (ev1, ev2)
   - Check if ev1 or ev2's clearance circle INTERSECTS the line t1→t2
   - If edge cuts through segment (v1wind != v2wind):
     - Use `check_intersect_vertex()` to measure violation depth
   - If edge is on one side:
     - Use `check_non_intersect_vertex()` to measure violation depth
   - Collect all violations as `rubberband_arc` candidates
4. Sort by violation depth (worst first)
5. Take the worst violation:
   - Create a new ARC around that obstacle vertex at clearance radius
   - Calculate tangent points (entry/exit) to connect with t1 and t2
6. RECURSE:
   - `oproute_rubberband_segment(path, t1, new_arc)` for left sub-segment
   - `oproute_rubberband_segment(path, new_arc, t2)` for right sub-segment
7. Return concatenated list of arcs

## Phase 5: Export (`toporouter_export()`)

For each routed net:
1. Call `oproute_rubberband()` to create arc representation
2. Call `export_oproutes()` to convert arcs to PCB line/arc objects

## Key Differences from Our Current Implementation

1. **Route vertices live ON edges** — not separate crossing records
2. **Winding check** prevents routes from exploring same-side triangles
3. **candidate_vertices** finds gaps between existing route vertices
4. **space_edge** is spring-based relaxation, not even distribution
5. **Rubber-banding creates ARC objects** — not path point manipulation
6. **The path IS the edge routing list** — no sync needed
