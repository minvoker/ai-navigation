"""
COS30019 Assignment 1 - Robot Navigation Search
Author: Max Harrison 104586300

HOW TO USE CODE INSTRUCTIONS:

"""

import sys
import json
from collections import deque 
from queue import PriorityQueue
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
    '''
    Depth-First Search (DFS) with explored set. This implementation uses a stack to explore the search tree depth-first, tracking explored states (set) to avoid revisiting them. Search terminates when goal state is found or entire search space has been explored.
    
    Takes an instance of Problem class as input and returns a tuple containing the goal node and the number of nodes expanded during the search. Returns None if no goal state is found.
    '''
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
    '''
    Uses a queue (deque) to explore the search tree level by level, tracking explored states (set) to avoid revisiting them. Search terminates when goal state is found or entire search space has been explored.
    
    Takes an instance of Problem class as input and returns a tuple containing the goal node and the number of nodes expanded during the search. Returns None if no goal state is found.
    '''
    node = Node(problem.initial)
    num_of_nodes = 1  # Count the initial node
    if problem.goal_test(node.state):
        return node, num_of_nodes
    
    frontier = deque([node])  # FIFO queue
    explored = set()

    while frontier:
        node = frontier.popleft()
        num_of_nodes += 1  # Increment counter for each child node
        explored.add(node.state)  
        if problem.goal_test(node.state):
            return node, num_of_nodes
        
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier: 
                frontier.append(child)
    
    return None, num_of_nodes

# Iterative Deepening CUS1 --------- WORKING
def iterative_deepening_bfs(problem):
    """Iterative Deepening Breadth-First Search"""
    depth = 0
    while True:
        result, num_nodes = depth_limited_bfs(problem, depth)
        if result is not None:
            return result, num_nodes
        depth += 1

def depth_limited_bfs(problem, limit):
    """Breadth-First Search with Depth Limit"""
    frontier = deque([(Node(problem.initial), 0)])  # (node, depth)
    explored = set()
    num_nodes = 0

    while frontier:
        node, depth = frontier.popleft()
        num_nodes += 1
        
        if problem.goal_test(node.state):
            return node, num_nodes
        explored.add(node.state)

        if depth < limit:
            for child in node.expand(problem):
                if child.state not in explored and child.state not in [n.state for n, d in frontier]:
                    frontier.append((child, depth + 1))

    return None, num_nodes

# ______________________________________________________________________________
# Informed Search algorithms
 # GBFS --------- WORKING
def greedy_best_first_search(problem):
    '''
    Perform a greedy best-first search on a problem with explored set. This implementation uses a list as the frontier, which is sorted by the heuristic value of each node. The node with the lowest heuristic value (Manhattan Distance) popped from the front for expansion. Explored set is used to avoid revisiting states. Terminates when goal state is found or entire search space has been explored.
    
    Takes an instance of Problem class as input and returns a tuple containing the goal node and the number of nodes expanded during the search. Returns None if no goal state is found.
    '''
    node = Node(problem.initial)
    num_of_nodes = 1  # Count the initial node
    if problem.goal_test(node.state):
        return node, num_of_nodes

    frontier = PriorityQueue(order='min', f=lambda x: x[0])
    frontier.append((problem.manhattan_distance(node.state), node))
    explored = set()

    while frontier.heap:
        h_n, node = frontier.pop()

        if node.state not in explored:
            num_of_nodes += 1
            explored.add(node.state)
            if problem.goal_test(node.state):
                return node, num_of_nodes

            for child in node.expand(problem):
                child_h_n = problem.manhattan_distance(child.state)
                if child.state not in explored:
                    frontier.append((child_h_n, child))

    return None, num_of_nodes

# AS --------- WORKING
def a_star_search(problem):
    '''
    Perform A* search on given problem with an explored set. This implementation uses a PriorityQueue as the frontier, which stores tuples of (f(n), node) where f(n) = g(n) + h(n). g(n) is the path cost and h(n) is the heuristic estimate (Manhattan Distance) of the cost from n to the goal state. At each iteration the node with lowest f(n) is popped from the front for expansion. Explored set is used to avoid revisiting states. Terminates when goal state is found or entire search space has been explored.
    
    Requires an instance of the problem class as input and returns a tuple containing the goal node and the number of nodes expanded during the search. Returns None if no goal state is found.
    '''
    node = Node(problem.initial)
    num_of_nodes = 1  # Count the initial node
    if problem.goal_test(node.state):
        return node, num_of_nodes

    frontier = PriorityQueue(order='min', f=lambda x: x[0])
    frontier.append((node.path_cost + problem.manhattan_distance(node.state), node))
    explored = set()

    while frontier.heap:
        f_n, node = frontier.pop()

        if node.state not in explored:
            num_of_nodes += 1
            explored.add(node.state)
            if problem.goal_test(node.state):
                return node, num_of_nodes

            for child in node.expand(problem):
                child_f_n = child.path_cost + problem.manhattan_distance(child.state)
                if child.state not in explored:
                    frontier.append((child_f_n, child))

    return None, num_of_nodes
   
# CUS2 --------- NOT WORKING

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
        # PRIORITY:  UP, LEFT, DOWN, RIGHT    # Flip for DFS cos Stack.
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
    
    def manhattan_distance(self, state):
        """Calculate the minimum Manhattan distance from a state to the nearest goal."""
        x1, y1 = state
        min_distance = float('inf')

        for goal_state in self.goal:
            x2, y2 = goal_state
            distance = abs(x1 - x2) + abs(y1 - y2)
            min_distance = min(min_distance, distance)

        return min_distance


def runRobotNavigation(filename, method):
    with open(filename, 'r') as f:
        data = [line.strip() for line in f.readlines()]
        f.close()
    
    rows, cols = eval(data[0])
    initial = eval(data[1])
    goal_coords = data[2].split('|')
    if len(goal_coords) == 1:
        goal = [eval(goal_coords[0])]
    else:
        goal = [eval(coord.strip()) for coord in goal_coords]

    # Create an empty grid filled with '.' (empty cells)
    grid = [['.' for _ in range(cols)] for _ in range(rows)]
    
    # Add Walls as '#'
    for i in data[3:]:
        if i: # Check line is not empty
            x, y, width, height = map(int, i.strip()[1:-1].split(','))
            for r in range(y, y + height):
                for c in range(x, x + width):
                    grid[r][c] = '#'
    
    prob = RobotNavigation(initial, goal, grid)
    result = None
    number_of_nodes = 0
    ''' # DEBUG: Display Grid
    print("Grid")          
    for i in grid:
        print(i, "\n")
    for goal in prob.goal:
        print("Goal", goal)
    '''
    
     # Search Algorithms____________________________________________
    if method.upper() == 'BFS':
        result, number_of_nodes = breadth_first_search(prob)
        
    elif method.upper() == 'DFS':
        result, number_of_nodes = depth_first_search(prob)
        
    elif method.upper() == 'GBFS':
        result, number_of_nodes = greedy_best_first_search(prob)
        
    elif method.upper() == 'AS':
        result, number_of_nodes = a_star_search(prob)
    
    elif method.upper() == 'CUS1':
        result, number_of_nodes = iterative_deepening_bfs(prob)
    
    elif method.upper() == 'CUS2':
        result, number_of_nodes = iterative_deepening_bfs(prob)
    # ______________________________________________________________
    
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
            
    
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python search.py <filename> <method>")
        sys.exit(1)

    filename = sys.argv[1]
    method = sys.argv[2]
    runRobotNavigation(filename, method)


