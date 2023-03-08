import heapq
import string
import time
import random
import logging
import turtle

from matplotlib import pyplot as plt
from src.maze import Maze
import numpy as np

# logging.basicConfig(level=logging.DEBUG)
max_iteration = 200
# threshold = 1e-8
threshold = 1e-3
discount = .99
reward_step = 0.0
final_reward = 1


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

    def backtrace(parent, start, end):
        l = list()
        path = [end]
        while path[-1] != start:
            # print(path[-1])
            path.append(parent[path[-1]])
            l.append((path[-1], False))            

        # print(l)
        return l

    def solve(self):
        logging.debug("Class BreadthFirst solver called")
        curr = [self.maze.entry_coor]
        path = list()
        parent = {}
        time_start = time.time()
        while True:
            next = list()
            while curr:

                x_coor, y_coor = curr.pop()
                self.maze.grid[x_coor][y_coor].visited = True
                path.append(((x_coor, y_coor), True))

                # Found the exit
                if(x_coor, y_coor) == self.maze.exit_coor:
                    path.append(((x_coor, y_coor), False))
                    path.extend(BreadthFirst.backtrace(parent ,self.maze.entry_coor, self.maze.exit_coor  ))
                    
                    
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
                        parent[neighbour] = (x_coor, y_coor)

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

    
    def solve(self):
        logging.debug("Class MDP solve called")
        if not self.quiet_mode:
            print("\nSolving the maze with MDP Value search...")

        exit_coor = (self.maze.exit_coor)
        num_rows = self.maze.num_rows
        num_cols = self.maze.num_cols
    	
  
        time_start = time.time()

        V = np.zeros((num_rows, num_cols))
        V[exit_coor] = final_reward

        delta = 0.0
        iteration = 0

        while iteration < max_iteration:
            delta = 0.0
            for i in range(num_rows): #maze_State[0]
                for j in range(num_cols):  #maze_State[1]]
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
        
        # Find the optimal policy
        policy = np.zeros((num_rows,num_cols), dtype=int)

        for i in range(num_rows):
            for j in range(num_cols):
                if True:
                    neighbours = self.maze.find_neighbours(i, j) 
                    neighbours = self.maze.validate_neighbours_solve(neighbours, i, j, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)
                   
                    action_values = {n: V[n] for n in neighbours}
                    next_cell = max(action_values, key=action_values.get)
                    diff_cell = ((next_cell[0] - i), ( next_cell[1] - j))

                    match diff_cell:
                        case (1, 0):
                            policy[i][j] = 0 # "up"
                        case (-1, 0):
                            policy[i][j] = 1 #"down"
                        case (0, -1):
                            policy[i][j] = 2 #"left"
                        case (0, 1):
                            policy[i][j] = 3 #"right"
        
        
        search_time = time.time() - time_start
        print("Time:               ", format(search_time))
        print("Value function:")
        flipped_v = np.flip(V, axis=0)
        formatted_v = np.array2string(flipped_v, precision=3, separator=',', suppress_small=True)

        # Print the formatted array
        print(formatted_v)

        print("Optimal policy:")
        flipped_policy = np.flip(policy, axis=0)
        print(flipped_policy)

        return V, policy

class PolicyIteration(Solver):

    def __init__(self, maze, quiet_mode=False,  neighbor_method = "brute-force"):
        logging.debug('Class MDP  policy iteration ctor called')

        super().__init__(maze, neighbor_method, quiet_mode)
        self.name = "MDP policy Search"
        

    def policy_eval(self, policy):
        num_rows = self.maze.num_rows
        num_cols = self.maze.num_cols
        V = np.zeros((num_rows, num_cols))
        V[self.maze.exit_coor] = final_reward

        iterations = 0
        while iterations < max_iteration:
            
            delta = 0.0
            for j in range(num_cols): 
                for i in range(num_rows): 
                    state = (i,j)
                    if (i, j) == self.maze.exit_coor:
                        continue
                    v = V[state]

                    next_i = i
                    next_j = j
                    match policy[i][j]:
                        case 0: 
                            # (1, 0): up
                           next_i = min(i + 1, num_rows - 1)
                        case 1: # (-1, 0): down
                            next_i = max(i - 1, 0)
                        case 2: # (0, -1):
                           next_j = max(j - 1 , 0)#"left"
                        case 3: #(0, 1):
                           next_j = min(j + 1, num_cols - 1) # right                 
                    
                    neighbours = self.maze.find_neighbours(i, j) 
                    neighbours = self.maze.validate_neighbours_solve(neighbours, i, j, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)
                    if (next_i, next_j) not in neighbours:
                        next_i = i
                        next_j = j

                    V[state] = reward_step + discount * V[(next_i, next_j)]
                    
                    delta = max(delta, abs(v - V[state]))
            iterations += 1
            if delta < threshold:
                break
        
        return (V)


    def solve(self):
        logging.debug("Class MDP policy solve called")
        if not self.quiet_mode:
            print("\nSolving the maze with MDP policy search...")

        num_rows = self.maze.num_rows
        num_cols = self.maze.num_cols
        
        time_start = time.time()

        policy = np.zeros((num_rows,num_cols), dtype=int)
        iteration = 0
        
        while iteration < max_iteration :
            V = PolicyIteration.policy_eval(self, policy)
            stable = True
            
            for j in range(num_cols): #maze_State[0]
                for i in range(num_rows):  #maze_State[1]]
                    
                    state = (i, j)
                
                    neighbours = self.maze.find_neighbours(i, j) 
                    neighbours = self.maze.validate_neighbours_solve(neighbours, i, j, self.maze.exit_coor[0], self.maze.exit_coor[1], self.neighbor_method)
                    
                    action_values = {n: V[n] for n in neighbours}
                    next_cell = max(action_values, key=action_values.get)
                    diff_cell = ((next_cell[0] - i), ( next_cell[1] - j))

                    match diff_cell:
                        case (1, 0):
                            best_action = 0 # "up"
                        case (-1, 0):
                            best_action = 1 #"down"
                        case (0, -1):
                            best_action = 2 #"left"
                        case (0, 1):
                            best_action = 3 #"right"
    
                    if best_action != policy[state]:
                        stable = False
                        policy[state] = best_action              

            if stable:
                break
            
            iteration = iteration + 1
        print("iterations:  ", iteration)

        search_time = time.time() - time_start
        print("Time:               ", format(search_time))

        print("Value function:")
        flipped_v = np.flip(V, axis=0)
        formatted_v = np.array2string(flipped_v, precision=3, separator=',', suppress_small=True)
        print(formatted_v)


        print("Optimal policy:")
        flipped_policy = np.flip(policy, axis=0)
        print(flipped_policy)


        return V, policy