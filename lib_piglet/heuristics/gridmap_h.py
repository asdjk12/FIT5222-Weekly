_HEUR_MODE = "dh"   # "dh" / "perfect"
_HEUR_K = 8
def set_heuristic_mode(mode: str, k: int | None = None):
    global _HEUR_MODE, _HEUR_K
    _HEUR_MODE = mode
    if k is not None: _HEUR_K = k

# heuristics/gridmap_h.py
#
# Heuristics for gridmap.
#
# @author: mike
# @created: 2020-07-22
#

import math, random
from lib_piglet.search.search_node import compare_node_g, compare_node_f, search_node
from lib_piglet.utils.data_structure import bin_heap
from lib_piglet.domains.gridmap import gridmap
from lib_piglet.search.dijkstra_search import dijkstra_search
from lib_piglet.search.graph_search import graph_search


pivot = {} # week 4
_PERFECT_TABLE = {}
PIVOT_SEED = 1234

def piglet_heuristic(domain,current_state, goal_state):
    if _HEUR_MODE == "perfect":
        return perfect_distance_heuristic(domain, current_state, goal_state)
    #return differential_heuristic(domain, current_state, goal_state, num_pivots=_HEUR_K)
    return manhattan_heuristic(current_state, goal_state)

def pigelet_multi_agent_heuristic(domain,current_state, goal_state):
    h = 0
    for agent, loc in current_state.agent_locations_.items():
        h += differential_heuristic(domain, loc, goal_state.agent_locations_[agent])
    return h

def manhattan_heuristic(current_state, goal_state):
    return abs(current_state[0]-goal_state[0]) + abs(current_state[1] - goal_state[1])

def straight_heuristic(current_state, goal_state):
    temp = math.pow(current_state[0]-goal_state[0],2) + math.pow(current_state[1]-goal_state[1],2) 
    return math.sqrt(temp)

def octile_heuristic(current_state, goal_state):
    distance = (abs(current_state[0]-goal_state[0]), abs(current_state[1]-goal_state[1]))
    temp = math.sqrt(2) * min(distance) + max(distance) - min(distance)
    return temp

def differential_heuristic(domain, current_state, goal_state, num_pivots):
    h = 0 # heurstic
    global pivot

    while len(pivot) < num_pivots:
        rng = random.Random(PIVOT_SEED + len(pivot))
        width = rng.randrange(0, domain.width_)
        height = rng.randrange(0, domain.height_)
        if domain.get_tile((height, width)):
            pivot[(height, width)] = None  # store distacne
    
    # solve SSSP problem, distance by dijkstra search
    for p, table  in list(pivot.items()):
        if table is None:
            open_list_ = bin_heap(compare_node_g)
            ds = dijkstra_search(open_list_, domain.expander_)
            # ds.expander_ = domain.expander_
            
            sol = ds.get_path(p)
            dist = getattr(sol, "path_", None) or getattr(sol, "solution_", None) or getattr(sol, "paths_", None) or sol
            pivot[p] = dist
    
    # calculate h
    for p, distance_table in pivot.items():
        distance_p_s = distance_table.get(current_state)
        distance_p_g = distance_table.get(goal_state)

        if distance_p_s == None or distance_p_g == None:
            continue

        dh = abs(distance_p_s-distance_p_g)
        if dh >h:
            h = dh
    return h 

def _perfect_distacne(domain, goal_state):
    global _PERFECT_TABLE
    if _PERFECT_TABLE is not None and _PERFECT_TABLE.get("__goal__") == goal_state:
        return
    
    open_list_ = bin_heap(compare_node_g)
    ds = dijkstra_search(open_list_, domain.expander_)
    sol = ds.get_path(goal_state)     
    dist_table = getattr(sol, "path_", None) or getattr(sol, "solution_", None) or getattr(sol, "paths_", None) or sol
    _PERFECT_TABLE = {"__goal__": goal_state}
    _PERFECT_TABLE.update(dist_table)

def perfect_distance_heuristic(domain, current_state, goal_state):
    fixed_goal = goal_state
    _perfect_distacne(domain, fixed_goal)
    return _PERFECT_TABLE.get(current_state, 0.0)

def compare(map, goal_state, num_pivots, num_rounds, seed):
    gm = gridmap(map)
    
    # random start point
    start = []
    rng = random.Random(seed)
    while len(start) < num_rounds:
        width = rng.randrange(0, gm.width_)
        height = rng.randrange(0, gm.height_)
        if gm.get_tile((height,width)):
            start.append((height,width))
    
    # genelise _PERFECT_TABLE for error calcluation and perfect-heurstic
    _perfect_distacne(gm, goal_state)
    true_shortest_distance = _PERFECT_TABLE

    dh_nodes, dh_time = 0, 0.0 
    pf_nodes, pf_time = 0, 0.0 
    err_sum = 0.0 
    err_count = 0

    # A* cost in DH
    for s in start:

        open1 = bin_heap(compare_node_f)
        set_heuristic_mode("dh", k=num_pivots)
        a1 = graph_search(bin_heap(compare_node_f), gm.expander_)
        _ = a1.get_path(s, goal_state)
        dh_nodes += a1.nodes_expanded_
        dh_time  += a1.runtime_

        # A*: Perfect
        open2 = bin_heap(compare_node_f)
        set_heuristic_mode("perfect")
        a2 = graph_search(open2, gm.expander_)
        _ = a2.get_path(s, goal_state)
        pf_nodes += a2.nodes_expanded_
        pf_time  += a2.runtime_

        # error
        h_star = true_shortest_distance.get(s)
        if h_star is not None:
            h_dh = differential_heuristic(gm, s, goal_state, num_pivots)
            if h_star >= h_dh:           
                err_sum += (h_star - h_dh)
                err_count += 1

    dh_avg_error = (err_sum / float(err_count)) if err_count > 0 else None
    
    return {
        "pivots": num_pivots,
        "DH_nodes_avg": dh_nodes / num_rounds,
        "DH_time_avg":  dh_time  / num_rounds,
        "PF_nodes_avg": pf_nodes / num_rounds,
        "PF_time_avg":  pf_time  / num_rounds,
        "DH_avg_error": dh_avg_error,
    }
    
if __name__ == "__main__":
    MAP = r"D:\Monash\FIT 5222\piglet-public-weekly\example\gridmap\arena2.map"
    GOAL = (10, 10)             
    for k in [1, 2, 4, 8, 16]:
        stats = compare(MAP, GOAL, num_pivots=k, num_rounds=200, seed=42)
        print(f"pivots={k} -> {stats}")
    