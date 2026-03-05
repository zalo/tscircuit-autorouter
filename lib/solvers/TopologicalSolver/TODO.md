# Topological Router Port — Remaining Work

## Tracing (High Priority)
- [x] `triangle_candidate_points_from_edge()` — parent/child gap tracing (ported)
- [x] `edge_adjacent_vertices()` — find adjacent committed vertices (ported)
- [ ] `triangle_candidate_points_from_vertex()` — full version with e1/e2 tracing
  - Currently: generates candidates on opposite edge without checking adjacent edges
  - gEDA: checks routing on e1/e2 to find vv1/vv2 gap boundaries on op_e
  - Impact: better gap finding when expanding from CDT vertices
- [ ] `edge_flow()` with proper per-vertex thickness/keepaway
  - Currently: uniform spacing (minTraceWidth + margin)
  - gEDA: min_spacing(v1, dest) uses actual net thickness and keepaway per vertex
- [ ] `check_triangle_interior_capacity()` — verify triangle has room for route
  - gEDA checks perpendicular distance from vertex to opposite edge vs flow

## Net Ordering (Medium Priority)
- [ ] `netscore_create()` — route each net in isolation, record score
- [ ] `netscore_pairwise_calculation()` — measure detour when nets coexist
- [ ] `order_nets_preroute_greedy()` — sort by pairwise fails then detour sum

## ROAR Rip-up (Medium Priority)
- [ ] `vertices_routing_conflicts()` — detect which routes conflict with a new route
- [ ] `roar_route()` — route with LEASTINVALID flag, detect conflicts, rip up, re-route
- [ ] Rollback mechanism (route_checkpoint / route_restore)
- [ ] `detour_router()` — optimize routes with excessive detours

## Rubber-banding (Low Priority — mostly working)
- [ ] `oproute_path_speccut()` — special cuts for tight triangle corridors
- [ ] `oproute_check_all_loops()` — comprehensive loop detection after all arcs
- [ ] `check_adj_pushing_vertex()` — check vertices adjacent to terminals
