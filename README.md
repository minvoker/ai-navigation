![Robot Navigation](/GUI.png)

## Table of Contents
- [Robot Navigation Problem](#robot-navigation-problem)
- [Search Algorithms](#search-algorithms)
- [Testing](#testing)
  - [Test Cases](#test-cases)
  - [Structure of the txt Files](#structure-of-the-txt-files)
- [Features/Missing](#featuresmissing)
  - [Features](#features)
  - [Missing Features](#missing-features)
- [Research](#research)
  - [Graphical User Interface](#graphical-user-interface)
- [Conclusion](#conclusion)
- [Acknowledgements/Resources/References](#acknowledgementsresourcesreferences)
- [Instructions](#instructions)

## Robot Navigation Problem
The Robot Navigation Problem is a search problem on an NxM grid with walls occupying some cells. A "Robot" agent is placed on the grid and must find a path to one of the predefined goal cells. The challenge is to navigate efficiently, avoiding walls and finding an optimal path to the goal.

## Search Algorithms

### Uninformed Search
- **Breadth-First Search (BFS)**: Explores all neighboring nodes at the current depth before moving to nodes at the next level, ensuring an optimal path but with high memory usage. Space complexity is O(b^d).
- **Depth-First Search (DFS)**: Explores a branch deeply before backtracking, using less memory but not guaranteeing an optimal path. Space complexity is O(m).
- **Iterative Deepening Breadth-First Search (CUS1)**: Combines BFS and DFS, performing depth-limited searches with increasing depth limits. It ensures an optimal path with space complexity similar to DFS.

### Informed Search
- **Manhattan Distance Heuristic**: Estimates the distance from a state to the goal, calculating the minimum number of vertical and horizontal moves required.
- **Greedy Best-First Search (GBFS)**: Uses a heuristic to estimate the cost from a node to the goal, prioritizing nodes closest to the goal. It finds solutions quickly but not necessarily optimal.
- **A* Search (AS)**: Combines BFS and GBFS strategies, using a heuristic and path cost to ensure an optimal solution if the heuristic is admissible.
- **Iterative Deepening A* Search (CUS2)**: A memory-efficient variant of A

## Testing

### Test Cases
Ten test cases are located in the `./tests` directory. These tests evaluate each algorithm's performance in various environments. Results are gathered and displayed using `test.py`.

### Structure of the txt Files

The txt files should follow this structure:

1. The first line contains the grid dimensions: `[N,M]` where `N` is the number of rows and `M` is the number of columns.
2. The second line contains the initial state of the agent: `(x,y)` where `x` and `y` are the coordinates of the initial position.
3. The third line contains the goal states for the agent, separated by `|`: `(x1,y1) | (x2,y2) | ...`.
4. Subsequent lines contain the walls in the format `(x,y,width,height)` where `(x,y)` is the top-left corner of the wall and `width` and `height` are the dimensions of the wall.

Example:
- [5,11] // The grid has 5 rows and 11 columns
- (0,1) // initial state of the agent – coordinates of the red cell
- (7,0) | (10,3) // goal states for the agent – coordinates of the green cells
- (2,0,2,2) // the square wall has the leftmost top corner occupies cell (2,0) and is 2 cells wide and 2 cell high
- (8,0,1,2)
- (10,0,1,1)
- (2,3,1,2)
- (3,4,3,1)
- (9,3,1,1)
- (8,4,2,1)

## Features/Missing

### Features
- **Visual Representation**: GUI using Pygame to display the grid and search path.
- **Test Script**: Compares performance metrics between search algorithms.

### Missing Features
- **Automatic Test Case Generation**: Creating random grid configurations for comprehensive testing.
- **Improved GUI**: Allowing search changes via mouse selection and displaying search steps with highlighted nodes.

### Graphical User Interface
Implemented using Pygame for simplicity and comprehensive documentation. Visual representation inspired by the assignment image, using a grid with distinct colors for the agent, walls, and goals.

## Acknowledgements/Resources/References
- **Artificial Intelligence: A Modern Approach** by Stuart Russel and Peter Norvig.

## Instructions

### Standard Output
To run the program and achieve standard output, follow these steps:

1. Open a command line interface (CLI).
2. Navigate to the directory containing `search.py`.
3. Execute the following command:
    ```
    python <filename> <method>
    ```
For example:
    ```
    python search.py RobotNav-Test.txt BFS
    ```

When a goal can be reached, the output will be:
- `filename method`
- `goal number_of_nodes`
- `path`

When no goal can be reached, the output will be:
- `filename method`
- `No goal is reachable; number_of_nodes`

### GUI Output
To use the visual version of this program, follow these steps:

1. Ensure Pygame is installed (`pip install pygame`).
2. Open a command line interface (CLI).
3. Navigate to the directory containing `search.py`.
4. Execute the following command with "GUI" at the end:
    ```
    python <filename> <method> GUI
    ```
For example:
    ```
    python search.py RobotNav-Test.txt BFS GUI
    ```

This will display a graphical representation of the grid and the search path.
