import os
from collections import defaultdict
from search import RobotNavigation, breadth_first_search, depth_first_search, greedy_best_first_search, a_star_search, iterative_deepening_bfs, iterative_deepening_astar

class TestSearch:
    def __init__(self, test_dir):
        self.test_dir = test_dir
        self.results = defaultdict(lambda: defaultdict(list))

    def run_tests(self):
        for filename in os.listdir(self.test_dir):
            if filename.endswith('.txt'):
                filepath = os.path.join(self.test_dir, filename)
                self.run_test_file(filepath)

        self.print_results()

    def run_test_file(self, filepath):
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

        for method in ['BFS', 'DFS', 'GBFS', 'AS', 'CUS1', 'CUS2']:
            result, num_nodes = self.run_algorithm(method, prob)
            if result is not None:
                path_length = len(result.solution())
                self.results[method]['num_nodes'].append(num_nodes)
                self.results[method]['path_lengths'].append(path_length)

    def run_algorithm(self, method, problem):
        if method == 'BFS':
            return breadth_first_search(problem)
        elif method == 'DFS':
            return depth_first_search(problem)
        elif method == 'GBFS':
            return greedy_best_first_search(problem)
        elif method == 'AS':
            return a_star_search(problem)
        elif method == 'CUS1':
            return iterative_deepening_bfs(problem)
        elif method == 'CUS2':
            return iterative_deepening_astar(problem)

    def print_results(self):
        print("Test Results:\n")
        for method, stats in self.results.items():
            num_nodes = stats['num_nodes']
            path_lengths = stats['path_lengths']
            avg_num_nodes = sum(num_nodes) / len(num_nodes) if num_nodes else 0
            avg_path_length = sum(path_lengths) / len(path_lengths) if path_lengths else 0
            print(f"{method}:")
            print(f"Average number of nodes generated: {avg_num_nodes:.2f}")
            print(f"Average path length: {avg_path_length:.2f}")
            print()

if __name__ == "__main__":
    dir = "./tests"
    tests = TestSearch(dir)
    tests.run_tests()