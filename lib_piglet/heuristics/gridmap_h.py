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

pivot = {} # week 4

def piglet_heuristic(domain,current_state, goal_state):
    return differential_heuristic(domain,current_state, goal_state)

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

def differential_heuristic(domain, current_state, goal_state):
    h = 0 # heurstic
    global pivot
    # choose 5 point randomly
    if not pivot:
        for i in range(7):
            width = random.randint(0, domain.width_ -1)
            height = random.randint(0, domain.height_ -1)
            if domain.get_tile((width, height)):
                pivot[(width,height)] = None  # store distacne
    
    # solve SSSP problem, distance by dijkstra search
    for p, table  in list(pivot.items()):
        if table is None:
            open_list_ = bin_heap(compare_node_f)
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