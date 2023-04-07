"""
Plan to make dynamic obstacles at some point
"""

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.trajectory = None

    def set_trajectory(self, trajectory):
        """
        Trajectory is stored as a dictionary of time: velocity. Only time stamps 
        where the velocity changes are stored. Missing time stamps are assumed to
        have the same velocity as the previous time stamp.
        """
        self.trajectory = trajectory

    def get_current_pos(self, time):
        """
        Get current position of obstacle
        """
        pass