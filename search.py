"""
COS30019 Assignment 1 - Robot Navigation Search
Author: Max Harrison 104586300

STEPS REQUIRED
1. Problem Setup
- Robot Navigation Search Problem
    NxM grid where N>1 and M>1
    
2. Search Strategies
Uninformed Search:
- Depth First Search (DFS)
- Breadth First Search (BFS)

Informed Search:
- Greedy Best First Search (GBFS)
- A* Search (AS)

Custom:
- ### UNINFORMED CUSTOM SEARCH (CUS1)
- ### INFORMED CUSTOM SEARCH (CUS2)

3. Command Line Operation
- python search.py <filename> <method>
- When a goal can be reached, standard output needs to be in the following format:
    filename method
    goal number_of_nodes
    path
- When a goal cannot be reached, standard output needs to be in the following format:
    filename method
    No goal is reachable; number_of_nodes

4. Report File


HOW TO USE CODE INSTRUCTIONS:

"""

import sys
import json
from collections import deque 

from utils import *


class Problem:
    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)


# ______________________________________________________________________________
# Uninformed Search algorithms
# Explored DFS --------- WORKING
def depth_first_search(problem):
    frontier = [(Node(problem.initial))]  # Stack
    num_of_nodes = 1  # Count the initial node
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node, num_of_nodes
        num_of_nodes += 1
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)
    return None, num_of_nodes

# Explored BFS --------- WORKING
def breadth_first_search(problem):
    """Breadth-First Search (BFS) with explored."""
    node = Node(problem.initial)
    num_of_nodes = 1  # Count the initial node
    if problem.goal_test(node.state):
        return node, num_of_nodes
    
    frontier_queue = deque([node])  # FIFO queue
    frontier_set = {(node.state[1], node.state[0])}  # For efficient membership checks, use swapped coordinates
    explored = set()

    while frontier_queue:
        node = frontier_queue.popleft()
        frontier_set.remove((node.state[1], node.state[0]))  
        explored.add((node.state[1], node.state[0]))  
        # DEBUG: print("Frontier: ", frontier_queue)
        for child in node.expand(problem):
            num_of_nodes += 1  # Increment counter for each child node
            if (child.state[1], child.state[0]) not in explored and (child.state[1], child.state[0]) not in frontier_set: 
                if problem.goal_test(child.state):
                    return child, num_of_nodes
                frontier_queue.append(child)
                frontier_set.add((child.state[1], child.state[0]))  

    return None, num_of_nodes

# ______________________________________________________________________________
# Informed Search algorithms

def depth_limited_search(problem, limit=50):
    """[Figure 3.17]"""

    def recursive_dls(node, problem, limit):
        if problem.goal_test(node.state):
            return node
        elif limit == 0:
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)


def iterative_deepening_search(problem):
    """[Figure 3.18]"""
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result


def uniform_cost_search(problem, display=False):
    """[Figure 3.14]"""
    #return best_first_search(problem, lambda node: node.path_cost, display)
    return 1

# ______________________________________________________________________________
# Robot Problem

class RobotNavigation(Problem):
    def __init__(self, initial, goal, grid):
        super().__init__(initial, goal)
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.goal = goal
        ''' CHECK GOAL TEST WORKING
        if initial not in goal:
                self.goal.append(initial)
                '''
    def goal_test(self, state):
        return state in self.goal
        
    def actions(self, state):
        col, row = state 
        actions = []
        # PRIORITY:  UP, LEFT, DOWN, RIGHT 
        directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]  
        for dir_col, dir_row in directions:  
            new_col = col + dir_col
            new_row = row + dir_row

            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                if self.grid[new_row][new_col] != '#':
                    actions.append((dir_col, dir_row)) 

        return actions

    def result(self, state, action):
        col, row = state  
        dir_col, dir_row = action  
        new_col, new_row = col + dir_col, row + dir_row 
        #DEBUG
        #print(f"Applying action {action} to state {state} results in state {(new_col, new_row)}")
        return (new_col, new_row)  


def runRobotNavigation(filename, method):
    with open(filename, 'r') as f:
        data = f.readlines()
    
    rows, cols = eval(data[0].strip())
    initial = eval(data[1].strip())
    goal = [eval(coord.strip()) for coord in data[2].strip().split('|')]
    # Unreachable GOAL:
    # goal = [20,9]
    # Create an empty grid filled with '.' (empty cells)
    grid = [['.' for _ in range(cols)] for _ in range(rows)]
    
    # Add Walls as '#'
    for i in data[3:]:
        x, y, width, height = map(int, i.strip()[1:-1].split(','))
        for r in range(y, y + height):
            for c in range(x, x + width):
                grid[r][c] = '#'
    '''
    print("Grid")          
    for i in grid:
        print(i, "\n")
    print("Goal State:", goal)
    '''
    prob = RobotNavigation(initial, goal, grid)
    result = None
    number_of_nodes = 0
    
    
    ''' Search Methods '''
    if method.upper() == 'BFS':
        result, number_of_nodes = breadth_first_search(prob)
        
    elif method.upper() == 'DFS':
        result, number_of_nodes = depth_first_search(prob)
    
    # ____________________________________________
    
    if result is None:
        print(f"{filename} {method}")
        print(f"No goal is reachable; {number_of_nodes} ")
    else:
        path = []
        pNode = result
        while pNode.parent:
            path.insert(0, pNode.action)
            pNode = pNode.parent
            
        directions = {(0, -1): 'up', (-1, 0): 'left', (0, 1): 'down', (1, 0): 'right'}

        path = [directions[action] for action in path]
        print(f"{filename} {method}\n{result} {number_of_nodes}\n{path}")
            
    
    


filename = 'RobotNav-test.txt'
runRobotNavigation(filename, 'DFS')


