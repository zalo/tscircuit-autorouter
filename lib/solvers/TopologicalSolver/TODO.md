# Topological Router Port — Remaining Work

## Tracing (High Priority)
- [ ] `triangle_candidate_points_from_edge()` — full parent/child gap tracing
  - Currently: simplified gap-finder looks at edge routing list directly
  - gEDA: traces parent/child links on adjacent edges e1/e2 to find where
    existing routes cross the opposite edge, determining correct gap boundaries
  - Impact: prevents two routes from using the same corridor without spacing
- [ ] `triangle_candidate_points_from_vertex()` — full version with e1/e2 tracing
  - Currently: generates candidates on opposite edge without checking adjacent edges
  - gEDA: checks routing on e1/e2 to find vv1/vv2 gap boundaries on op_e
  - Impact: better gap finding when routes cross adjacent triangles
- [ ] `edge_adjacent_vertices()` — find the route vertices adjacent to a given
  vertex on an edge (prev/next in the routing list that are non-temp)
  - Used by both triangle_candidate_points functions
- [ ] `edge_flow()` with proper per-vertex thickness/keepaway
  - Currently: uniform spacing (minTraceWidth + margin)
  - gEDA: min_spacing(v1, dest) uses actual net thickness and keepaway per vertex

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
