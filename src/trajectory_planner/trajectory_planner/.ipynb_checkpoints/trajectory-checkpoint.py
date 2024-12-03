import numpy as np

from slam import SLAM

class TrajectoryPlanner:
    """
    Base class for trajectory planning. Defines the interface and common functionality
    for trajectory planners.
    """
    def __init__(self, map: SLAM):
        self.trajectory: np.ndarray = np.array()
        self.map = map

    def generate_trajectory(self, start:np.ndarray, goal:np.ndarray):
        """
        Plan a trajectory from start to goal.

        This method should be implemented by subclasses.
        """
        raise NotImplementedError("The plan method must be implemented by subclasses.")

    def get_trajectory(self):
        """
        Get the planned trajectory.
        """
        return self.trajectory

class TrivialTrajectory(TrajectoryPlanner):
    """
    A simple implementation of a trajectory planner that moves directly from start to goal.
    """
    def __init__(self):
        pass

    def generate_trajectory(self, start:np.ndarray, goal:np.ndarray):
        """
        Plan a trajectory that goes directly from start to goal.
        """
        self.trajectory = np.array([start, goal])

class AStarTrajectory(TrajectoryPlanner):
    """
    A trajectory planner implementation using the A* search algorithm.
    """
    def __init__(self):
        pass

    def generate_trajectory(self, start:np.ndarray, goal:np.ndarray):
        """
        Plan a trajectory using the A* search algorithm.
        """
        self.trajectory = self._a_star_search(start, goal, self.map)

    def _a_star_search(self, start, goal, map):
        pass

# Example usage
if __name__ == "__main__":
    start = [0, 0]
    goal = [5, 5]

    # Using TrivialTrajectory
    trivial_planner = TrivialTrajectory()
    trivial_planner.plan(start, goal)
    print("Trivial Trajectory:", trivial_planner.get_trajectory())

    # Using AStarTrajectory
    grid_map = [[0] * 10 for _ in range(10)]  # Example grid map
    a_star_planner = AStarTrajectory(grid_map)
    a_star_planner.plan(start, goal)
    print("A* Trajectory:", a_star_planner.get_trajectory())
