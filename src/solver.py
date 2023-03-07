import heapq
import time
import random
import logging
import turtle

from matplotlib import pyplot as plt
from src.maze import Maze
import numpy as np

# logging.basicConfig(level=logging.DEBUG)


class Solver(object):
    """Base class for solution methods.
    Every new solution method should override the solve method.

    Attributes:
        maze (list): The maze which is being solved.
        neighbor_method:
        quiet_mode: When enabled, information is not outputted to the console

    """

    def __init__(self, maze, quiet_mode, neighbor_method):
        logging.debug("Class Solver ctor called")

        self.maze = maze
        self.neighbor_method = neighbor_method
        self.name = ""
        self.quiet_mode = quiet_mode

    def solve(self):
        logging.debug('Class: Solver solve called')
        raise NotImplementedError

    def get_name(self):
        logging.debug('Class Solver get_name called')
        raise self.name

    def get_path(self):
        logging.debug('Class Solver get_path called')
        return self.path

class BreadthFirst(Solver):

    def __init__(self, maze, quiet_mode, neighbor_method):
        logging.debug('Class BreadthFirst ctor called')

        self.name = "Breadth First Recursive"
        super().__init__(maze, neighbor_method, quiet_mode)

    def solve(self):
        logging.debug("Class BreadthFirst solver called")
        curr = [self.maze.entry_coor]
        path = list()
        time_start = time.time()
        while True:
            next = list()
            while curr:

                x_coor, y_coor = curr.pop()
                self.maze.grid[x_coor][y_coor].visited = True
                path.append(((x_coor, y_coor), True))

                # Found the exit
                if(x_coor, y_coor) == self.maze.exit_coor:
                    search_time = time.time() - time_start
                    print("Found path using BFS")
                    print("Time:               ", format(search_time))
                    print("Path Length:        ", format(len(path)))
                    return path
                
                # Didn't find the exit
                neighbours = self.maze.find_neighbours(x_coor, y_coor) 
                neighbours = self.maze.validate_neighbours_solve(neighbours, x_coor, y_coor, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)

                if neighbours != None:
                    for neighbour in neighbours:
                        next.append(neighbour)

            for cell in next:
                curr.append(cell)

class DepthFirst(Solver):

    def __init__(self, maze, quiet_mode,  neighbor_method):
        logging.debug('Class DepthFirstSearch ctor called')

        super().__init__(maze, neighbor_method, quiet_mode)
        self.name = "Depth First Search"

    # Function to solve the maze using depth-first search
    def solve(self):
        visited = list()
        path = list()

        time_start = time.time()
        x_coor, y_coor = self.maze.entry_coor
        self.maze.grid[x_coor][y_coor].visited = True
        

        while True:

            # Found the exit
            if(x_coor, y_coor) == self.maze.exit_coor:
                path.append(((x_coor, y_coor), False))
                self.maze.grid[x_coor][y_coor].visited = True
                search_time = time.time() - time_start
                print("Found path using DFS")
                print("Time:               ", format(search_time))
                print("Path Length:        ", format(len(path)))
                return path
            
            neighbours = self.maze.find_neighbours(x_coor, y_coor) 
            neighbours = self.maze.validate_neighbours_solve(neighbours, x_coor, y_coor, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)


            if neighbours != None:
                visited.append((x_coor, y_coor))
                path.append(((x_coor, y_coor), False))
                x_coor, y_coor = random.choice(neighbours)
                self.maze.grid[x_coor][y_coor].visited = True

            else:
                path.append(((x_coor, y_coor), True))
                x_coor, y_coor = visited.pop()
        
class AStar(Solver):

    def __init__(self, maze, quiet_mode,  neighbor_method):
        logging.debug('Class AStarSearch ctor called')

        super().__init__(maze, neighbor_method, quiet_mode)
        self.name = "A* Search"

    
    def heuristic(cell_a, cell_b):    
        return (abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1]))

    def solve(self):
        logging.debug("Class DepthFirstBacktracker solve called")
        if not self.quiet_mode:
            print("\nSolving the maze with depth-first search...")

        start_x_coor, start_y_coor = self.maze.entry_coor
        exit_x_coor, exit_y_coor = self.maze.exit_coor
        start_coor = (start_x_coor, start_y_coor)
        exit_coor = (exit_x_coor, exit_y_coor)

        path = list()

        open_set = []
        visited = set()
        heapq.heappush(open_set, (AStar.heuristic(start_coor, exit_coor), start_coor, 0, None))
 

        time_start = time.time()

        while open_set:
            _, current_pos, g_score, parent = heapq.heappop(open_set)
           
            

            self.maze.grid[current_pos[0]][current_pos[1]].visited = True
            path.append(((current_pos[0], current_pos[1]), True))

            if current_pos == exit_coor:
                # path.append(((x_coor, y_coor), False))
                # self.maze.grid[x_coor][y_coor].visited = True
                search_time = time.time() - time_start
                print("Found path using A*")
                print("Time:               ", format(search_time))
                print("Path Length:        ", format(len(path)))
                return path

       
        
            # Explore the neighbors of the current node
            neighbours = self.maze.find_neighbours(current_pos[0], current_pos[1]) 
            neighbours = self.maze.validate_neighbours_solve(neighbours, current_pos[0], current_pos[1], exit_coor[0], exit_coor[1], self.neighbor_method)
            
            if neighbours == None:
                continue
            for neighbor in neighbours:
                # print(neighbor)
                
                # Check if the neighbor node is already in the closed set
                if neighbor in visited:
                    continue

                visited.add(current_pos)

                new_g_score = g_score + 1
                f_score = new_g_score + AStar.heuristic(neighbor, exit_coor)

                found = False
                for i, (f, n, _, _) in enumerate(open_set):
                    if neighbor == n:
                        found = True
                        # Update the neighbor node's g-score and f-score if it's lower than the current value
                        if new_g_score < open_set[i][2]:
                            open_set[i] = (f_score, neighbor, new_g_score, (neighbor, _, _, parent))
                            heapq.heapify(open_set)
                        break
                if not found:
                    heapq.heappush(open_set, (f_score, neighbor, new_g_score, (neighbor, _, _, parent)))

        if not self.quiet_mode:
            print('AStar leaving unsolved')
            print("Number of moves performed: {}".format(len(path)))
            print("Execution time for algorithm: {:.4f}".format(time.time() - time_start))
        print("test8")
        logging.debug('AStar leaving unsolved')
        return None

class MDP(Solver):
  
    
    def __init__(self, maze, quiet_mode=False,  neighbor_method = "brute-force"):
        logging.debug('Class MDP ctor called')

        super().__init__(maze, neighbor_method, quiet_mode)
        self.name = "MDP Search"

    # Function to solve the maze using depth-first search
    
    def solve(self):
        logging.debug("Class MDP solve called")
        if not self.quiet_mode:
            print("\nSolving the maze with depth-first search...")


        start_coor = (self.maze.entry_coor)
        exit_coor = (self.maze.exit_coor)
        num_rows = self.maze.num_rows
        num_cols = self.maze.num_cols
    	
        max_iteration = 200
        threshold = 1e-5

        discount = .9
        reward_step = 0
        final_reward = 1
        noise = 0.25
  
        time_start = time.time()

 # Perform value iteration until convergence
        V = np.zeros((num_rows, num_cols))
        V[exit_coor] = final_reward

        delta = 0.0
        iteration = 0

        while iteration < max_iteration:
            delta = 0.0
            for i in range(num_cols): #maze_State[0]
                for j in range(num_rows):  #maze_State[1]]
                    state = (i, j)
                    if state != exit_coor:
                        v = V[state]
                        q = []
                        neighbours = self.maze.find_neighbours(i, j) 
                        neighbours = self.maze.validate_neighbours_solve(neighbours, i, j, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)

                        for neighbour in neighbours:                          
                            q.append(reward_step + discount * V[neighbour])

                        V[state] = max(q)
                        delta = max(delta, abs(v - V[state]))
            
            if delta < threshold:
                print("converged")
                break
            
            iteration = iteration + 1
        print("iterations:  ", iteration)


        search_time = time.time() - time_start
        print("Time:               ", format(search_time))
        # Find the optimal policy
        policy = np.zeros((num_cols,num_rows), dtype=tuple)
        
        for i in range(num_cols):
            for j in range(num_rows):
                if True:
                # (i, j) != exit_coor:
                # if grid[i][j] != '#':
                    # Dictionary comprehension to get value associated with each action
                    neighbours = self.maze.find_neighbours(i, j) 
                    neighbours = self.maze.validate_neighbours_solve(neighbours, i, j, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)
                    # for neighbour in neighbours:
                   
                    action_values = {n: V[n] for n in neighbours}

                    policy[i][j] = max(action_values, key=action_values.get)
                
        

        print("Value function:")
        flipped_v = np.flip(V, axis=0)
        formatted_v = np.array2string(flipped_v, precision=3, separator=',', suppress_small=True)

        # Print the formatted array
        print(formatted_v)
        # print(V)

        print("Optimal policy:")
        flipped_policy = np.flip(policy, axis=0)
        print(flipped_policy)

        return V, policy
        
                

       
        # # logging.debug('Class MDP leaving solve')
        # return path


class PolicyIteration(Solver):

    def __init__(self, maze, quiet_mode=False,  neighbor_method="fancy"):
        logging.debug('Class DepthFirstSearch ctor called')

        super().__init__(maze, neighbor_method, quiet_mode)
        self.name = "Depth First Search"

    # Function to solve the maze using depth-first search
    
    def solve(self):
        logging.debug("Class DepthFirstBacktracker solve called")
        k_curr, l_curr = self.maze.entry_coor      # Where to start searching
        self.maze.grid[k_curr][l_curr].visited = True     # Set initial cell to visited
        visited_cells = list()                  # Stack of visited cells for backtracking
        path = list()                           # To track path of solution and backtracking cells
        if not self.quiet_mode:
            print("\nSolving the maze with depth-first search...")

        time_start = time.time()

        while (k_curr, l_curr) != self.maze.exit_coor:     # While the exit cell has not been encountered
            neighbour_indices = self.maze.find_neighbours(k_curr, l_curr)    # Find neighbour indices
            neighbour_indices = self.maze.validate_neighbours_solve(neighbour_indices, k_curr,
                l_curr, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)

            if neighbour_indices is not None:   # If there are unvisited neighbour cells
                visited_cells.append((k_curr, l_curr))              # Add current cell to stack
                path.append(((k_curr, l_curr), False))  # Add coordinates to part of search path
                k_next, l_next = random.choice(neighbour_indices)   # Choose random neighbour
                self.maze.grid[k_next][l_next].visited = True                 # Move to that neighbour
                k_curr = k_next
                l_curr = l_next

            elif len(visited_cells) > 0:              # If there are no unvisited neighbour cells
                path.append(((k_curr, l_curr), True))   # Add coordinates to part of search path
                k_curr, l_curr = visited_cells.pop()    # Pop previous visited cell (backtracking)

        path.append(((k_curr, l_curr), False))  # Append final location to path
        if not self.quiet_mode:
            print("Number of moves performed: {}".format(len(path)))
            print("Execution time for algorithm: {:.4f}".format(time.time() - time_start))

        logging.debug('Class DepthFirstBacktracker leaving solve')
        return path
