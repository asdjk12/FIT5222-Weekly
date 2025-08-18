from __future__ import annotations
# expander/grid_expander.py
# 
# Expand function for the 4-connected gridmap domain.
#
# Given a current search node, the expander checks the set of valid grid actions 
# and generates search node successors for each.
#
# @author: dharabor
# @created: 2020-07-15
#

from lib_piglet.search.search_node import search_node
from lib_piglet.expanders.base_expander import base_expander
from lib_piglet.domains.grid_action import  Move_Actions, grid_action
from lib_piglet.constraints.grid_constraints import grid_constraint_table, grid_reservation_table
from typing import TYPE_CHECKING

import copy
if TYPE_CHECKING:
    from lib_piglet.domains.gridmap import (
        gridmap,          
        gridmap_joint,
        grid_joint_state,
    )

class grid_expander(base_expander):
    def __init__(self, map : gridmap, constraint_table: grid_constraint_table = None):
        self.domain_: gridmap = map
        self.effects_: list = [self.domain_.height_*-1, self.domain_.height_, -1, 1]
        self.constraint_table_: grid_constraint_table   = constraint_table
        self.reservation_table_: grid_reservation_table = None # reservation_table_ is not used on default, decide how to use it on your own.

        # memory for storing successor (state, action) pairs
        self.succ_: list = [] 


    # identify successors of the current node
    #
    # @param current: The current node
    # @return : Possible next
    def expand(self, current: search_node):
        self.succ_.clear()
        curr_state = current.state_       

        for mv in self.get_actions(curr_state):
            nxt_state = self.__move(curr_state, mv)

            act = grid_action()           
            act.move_ = mv
            act.cost_ = 1                

            self.succ_.append((nxt_state, act))

        return self.succ_[:]              

    # return a list with all the applicable/valid actions
    # at tile (x, y)
    # @param loc A (x,y) coordinate tuple
    # @return a list of gridaction object.
    def get_actions(self, loc: tuple):
        x = loc[0]
        y = loc[1]
        W, H = self.domain_.width_, self.domain_.height_
        retval = []

        candidates = [
            Move_Actions.MOVE_UP,
            Move_Actions.MOVE_DOWN,
            Move_Actions.MOVE_LEFT,
            Move_Actions.MOVE_RIGHT,
            Move_Actions.MOVE_UP_LEFT,
            Move_Actions.MOVE_UP_RIGHT,
            Move_Actions.MOVE_DOWN_LEFT,
            Move_Actions.MOVE_DOWN_RIGHT,
        ]
        for ma in candidates:
            nx, ny = self.__move(loc, ma)

            # boundary
            if nx < 0 or ny < 0 or nx >= H or ny >= W:
                continue
            # block
            if not self.domain_.get_tile((nx, ny)):
                continue

            retval.append(ma)           
        return retval

    def __move(self, curr_state: tuple, move):
        x = curr_state[0]
        y = curr_state[1]

        if move == Move_Actions.MOVE_UP:        
            return (x-1, y)
        elif move == Move_Actions.MOVE_DOWN:      
            return (x+1, y)
        elif move == Move_Actions.MOVE_LEFT:      
            return (x, y-1)
        elif move == Move_Actions.MOVE_RIGHT:     
            return (x, y+1)
        elif move == Move_Actions.MOVE_UP_LEFT:     
            return (x-1, y-1)
        elif move == Move_Actions.MOVE_UP_RIGHT:     
            return (x-1, y+1)
        elif move == Move_Actions.MOVE_DOWN_LEFT:     
            return (x+1, y-1)
        elif move == Move_Actions.MOVE_DOWN_RIGHT:     
            return (x+1, y+1)
        else:                                     
            return (x,y)

    def __str__(self):
        return str(self.domain_)




class grid_joint_expander(base_expander):


    def __init__(self, map: gridmap, constraint_table: grid_constraint_table = None):
        self.domain_: gridmap_joint = map
        self.effects_: list = [self.domain_.height_ * -1, self.domain_.height_, -1, 1]
        self.constraint_table_: grid_constraint_table  = constraint_table
        self.reservation_table_: grid_reservation_table = None   # reservation_table_ is not used on default, decide how to use it on your own.

        # memory for storing successor (state, action) pairs
        self.succ_: list = []

    # identify successors of the current node
    #
    # @param current: The current node
    # @return : Possible next
    def expand(self, current: search_node):

        self.succ_.clear()
        current_state : grid_joint_state =  copy.deepcopy(current.state_)
        #################
        # Implement your codes to generate all possible child states (all possible combination of movements of all agents) here.
        #
        # Read the implementation of grid_joint_state, you can find grid_joint_state contains a dictionary agent_locations_
        # that stores the agent_id as key and its corresponding location.
        #
        # The domain_ is of type gridmap_joint, which also stores start_ and goal_ state, in case you want some
        # information from goal state for well formed instance practice..
        #################
        return self.succ_[:]
    
    def generate_states_recursively(self, current_state: grid_joint_state, agents_left: list, cost: int, loc_set, parent_state):
        ############
        # If you want to generate possible child states in a recursive manner, you might want to implement this function
        ############
        raise NotImplementedError

    # return a list with all the applicable/valid actions
    # at tile (x, y)
    # @param loc A (x,y) coordinate tuple
    # @return a list of gridaction object.
    def get_actions(self, loc: tuple):
        x = loc[0]
        y = loc[1]
        retval = []

        if (x < 0 or x >= int(self.domain_.height_) or y < 0 or y >= int(self.domain_.width_)):
            return retval

        if (self.domain_.get_tile(loc) == False):
            return retval

        if (self.domain_.get_tile((x, y - 1))):
            retval.append(grid_action())
            retval[-1].move_ = Move_Actions.MOVE_LEFT
            retval[-1].cost_ = 1

        if (self.domain_.get_tile((x, y + 1))):
            retval.append(grid_action())
            retval[-1].move_ = Move_Actions.MOVE_RIGHT
            retval[-1].cost_ = 1

        if (self.domain_.get_tile((x - 1, y))):
            retval.append(grid_action())
            retval[-1].move_ = Move_Actions.MOVE_UP
            retval[-1].cost_ = 1

        if (self.domain_.get_tile((x + 1, y))):
            retval.append(grid_action())
            retval[-1].move_ = Move_Actions.MOVE_DOWN
            retval[-1].cost_ = 1

        if (self.domain_.get_tile((x, y))):
            retval.append(grid_action())
            retval[-1].move_ = Move_Actions.MOVE_WAIT
            retval[-1].cost_ = 1

        return retval

    def __move(self, curr_state: tuple, move):
        x = curr_state[0]
        y = curr_state[1]
        if move == Move_Actions.MOVE_UP:
            x -= 1
        elif move == Move_Actions.MOVE_DOWN:
            x += 1
        elif move == Move_Actions.MOVE_LEFT:
            y -= 1
        elif move == Move_Actions.MOVE_RIGHT:
            y += 1

        return x, y

    def __str__(self):
        return str(self.domain_)
    
    