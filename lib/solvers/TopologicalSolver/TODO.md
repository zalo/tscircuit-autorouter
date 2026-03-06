# Topological Router Port — Status

## Completed

### Tracing
- [x] `triangle_candidate_points_from_edge()` — parent/child gap tracing on e1/e2
- [x] `triangle_candidate_points_from_vertex()` — e1/e2 tracing to find vv1/vv2 on op_e
- [x] `edge_adjacent_vertices()` — find adjacent committed vertices
- [x] `check_triangle_interior_capacity()` — capacity vs flow check
- [x] `candidate_vertices()` — gap-based with per-vertex spacing
- [x] `edge_flow()` with per-vertex thickness/keepaway
- [x] `space_edge()` — 100-iter force-based relaxation
- [x] Segment overlap prevention during A*
- [x] Steiner points (obstacle centers + connection endpoints)

### Rubber-banding
- [x] `oproute_rubberband_segment()` — recursive arc insertion
- [x] `check_intersect_vertex()` / `check_non_intersect_vertex()`
- [x] `calculate_term_to_arc()` / `calculate_arc_to_arc()`
- [x] `check_arc_for_loops()` — arc loop detection

### Net Ordering
- [x] `netscore_create()` — isolated routing score
- [x] `netscore_pairwise_calculation()` — pairwise detour measurement
- [x] `order_nets_preroute_greedy()` — sort by fails/detour/score

### ROAR Rip-up
- [x] `findConflictingRoutes()` — detect conflicting committed routes
- [x] `roarRoute()` — LEASTINVALID routing + conflict detection + rollback
- [x] `roarPass()` — iterative ROAR passes
- [x] `leastInvalidMode` flag

### Post-routing Optimization
- [x] `detour_router()` — rip up detoured routes and re-optimize

## Remaining (Low Priority)
- [ ] `oproute_path_speccut()` — special cuts for tight triangle corridors
- [ ] `oproute_check_all_loops()` — comprehensive loop detection after all arcs
- [ ] Full `vertices_routing_conflicts()` with split_edge_routing topology
- [ ] Cluster management (grouping terminals by electrical connectivity)

## Benchmark
- 92/100 fully routed (dataset-01)
- 2662/2678 connections (99.4%)
- 0 crossings
- 0 timeouts
