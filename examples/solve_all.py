from __future__ import absolute_import
from src.maze_manager import MazeManager
from src.maze import Maze
from copy import deepcopy


if __name__ == "__main__":

    # Create the manager


    # Add a 10x10 maze to the manager
    manager = MazeManager()

    maze = Maze(10, 10)
    maze2 = deepcopy(maze)
    maze3 = deepcopy(maze)
    maze4 = deepcopy(maze)
    maze5 = deepcopy(maze)
    
    maze = manager.add_existing_maze(maze)  
    manager.show_maze(maze.id)
    
    manager.solve_maze(maze.id, "BreadthFirstSearch")
    manager.show_solution_animation(maze.id)
    

    manager2 = MazeManager()
    maze2 = manager2.add_existing_maze(maze2) 
    manager2.solve_maze(maze2.id, "DepthFirstSearch")
    manager2.show_solution_animation(maze2.id)

    manager3 = MazeManager()
    maze3 = manager3.add_existing_maze(maze3) 
    manager3.solve_maze(maze3.id, "AStarSearch")
    manager3.show_solution_animation(maze3.id)

    manager4 = MazeManager()
    maze4 = manager4.add_existing_maze(maze4) 
    manager4.solve_maze(maze4.id, "MDPSearch")
    manager4.show_value_solution(maze4.id)
    manager4.show_policy_solution(maze4.id)

    manager5 = MazeManager()
    maze5 = manager5.add_existing_maze(maze5) 
    manager5.solve_maze(maze5.id, "PolicyIterationSearch")
    manager5.show_value_solution(maze5.id)
    manager5.show_policy_solution(maze5.id)
    