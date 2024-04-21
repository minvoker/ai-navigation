"""
COS30019 Assignment 1 - Robot Navigation Search
Author: Max Harrison 104586300

Test Class
HOW TO USE CODE INSTRUCTIONS:
Run the test.py file to run algorithms on each test case in ./tests.
"""
from search import RobotNavigation, breadth_first_search, depth_first_search, greedy_best_first_search, a_star_search, iterative_deepening_bfs, iterative_deepening_astar
from collections import defaultdict # For storing results
import os

class Test:
    def __init__(self, test_dir):
        self.test_dir = test_dir
        # Dictionary to store results for each algorithm
        self.results = defaultdict(lambda: defaultdict(list))

    def run_tests(self):
        for filename in os.listdir(self.test_dir):
            if filename.endswith('.txt'):
                #print("File found: ", filename)
                filepath = os.path.join(self.test_dir, filename)
                self.run_test_file(filepath)

        self.print_results()

    def run_test_file(self, filepath):
        # Setup identical to search.py
        with open(filepath, 'r') as f:
            data = [line.strip() for line in f.readlines()]
            f.close()
        rows, cols = eval(data[0])
        initial = eval(data[1])
        goal_coords = data[2].split('|')

        if len(goal_coords) == 1:
            goal = [eval(goal_coords[0])]
        else:
            goal = [eval(coord.strip()) for coord in goal_coords]

        grid = [['.' for _ in range(cols)] for _ in range(rows)]

        for i in data[3:]:
            if i:
                x, y, width, height = map(int, i.strip()[1:-1].split(','))
                for r in range(y, y + height):
                    for c in range(x, x + width):
                        grid[r][c] = '#'
        
        prob = RobotNavigation(initial, goal, grid)
        # Run each algorithm in order and append to results
        for search in ['BFS', 'DFS', 'CUS1', 'GBFS', 'AS', 'CUS2']:
            result, num_nodes = self.run_search(search, prob)
            
            if result is not None:
                path_length = len(result.solution())
                self.results[search]['num_nodes'].append(num_nodes)
                self.results[search]['path_lengths'].append(path_length)


    def run_search(self, search, problem):
        if search == 'BFS':
            return breadth_first_search(problem)
        elif search == 'DFS':
            return depth_first_search(problem)
        elif search == 'CUS1':
            return iterative_deepening_bfs(problem)
        elif search == 'GBFS':
            return greedy_best_first_search(problem)
        elif search == 'AS':
            return a_star_search(problem)
        elif search == 'CUS2':
            return iterative_deepening_astar(problem)

    def print_results(self):
        print("Test Results:")
        for search, stats in self.results.items():
            num_nodes = stats['num_nodes']
            path_lengths = stats['path_lengths']
            avg_num_nodes = sum(num_nodes) / len(num_nodes) if num_nodes else 0
            avg_path = sum(path_lengths) / len(path_lengths) if path_lengths else 0
            print("----------------------------------")
            print(f"{search}:")
            print(f"Average number of nodes generated: {avg_num_nodes:.2f}")
            print(f"Average path length: {avg_path:.2f}")


if __name__ == "__main__":
    dir = "./tests"
    tests = Test(dir)
    tests.run_tests()