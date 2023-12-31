from __future__ import absolute_import
from src.maze_manager import MazeManager


if __name__ == "__main__":

    # Create the manager
    manager = MazeManager()

    # Add a 10x10 maze to the manager
    maze = manager.add_maze(10, 10)

    # Solve the maze using the Breadth First algorithm
    manager.solve_maze(maze.id, "PolicyIterationSearch")

    # Display the maze
    manager.show_maze(maze.id)

    # Display the maze with the solution overlaid
    manager.show_value_solution(maze.id)

    manager.show_policy_solution(maze.id)
