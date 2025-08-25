# search/search_node.py
#
# Data structure that represents a domain-independent search node
#
# @author: dharabor
# @created: 2020-07-15
#

import sys, random
from functools import total_ordering
from typing import Generic, TypeVar

from lib_piglet.utils.identifier import get_random_id, identifier

State = TypeVar("State")


class search_node(Generic[State]):

    def __init__(self, name:str=None):
        self.id = name or identifier(self)
        # some default values for uninitialised nodes
        self.action_: object = None
        self.state_: State = None
        self.parent_: search_node[State] = None
        self.g_: float = 0
        self.depth_: int = 0
        self.instance_: int = 0

        self.h_: float = 0
        self.f_: float = 0
        self.timestep_: int = 0
        self.closed_: bool = False
        self.priority_queue_handle_: object = None
        self.expanded: bool = False

    # Is the node closed
    # @return bool True if the node is closed
    def is_closed(self):
        return self.closed_

    # Mark the node as closed
    def close(self):
        self.closed_ = True

    # Mark the node as open
    def open(self):
        self.closed_ = False

    def __str__(self):
        return str(self.state_)

    def __repr__(self):
        return self.state_.__repr__()

    def __eq__(self, other):
        if other == None:
            return False
        return self.state_ == other.state_

    def __hash__(self):
        return hash(self.state_)


# Compare two node by g value
# Return true if a >= b
def compare_node_g(a: search_node, b: search_node):
    return a.g_ >= b.g_


# Compare two node by f value
# Return true if a >= b
def compare_node_f(a: search_node, b: search_node):
    if a.f_ == b.f_:

        return a.h_ >= b.h_

        # largest-h
        return a.h_ >= b.h_
        # smallest-h
        #return a.h_ <= b.h_
        #random
        #return random.random() < 0.5

    return a.f_ >= b.f_


# Compare two node by h value
# Return true if a >= b
def compare_node_h(a: search_node, b: search_node):
    return a.h_ >= b.h_
