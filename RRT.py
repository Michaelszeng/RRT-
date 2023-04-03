"""
Helper functions and classes for RRT* algorithm
"""
import math
import pygame

class Window:
    def __init__(self, start, goal, map_dimensions, obstacles):
        self.width, self.height = map_dimensions
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

        # pygame settings
        pygame.display.set_caption("RRT*")
        self.window = pygame.display.set_mode((self.width, self.height))
        self.window.fill((0,0,0))
        self.node_radius = 10
        self.node_thickness = 0
        self.edge_thickness = 0

    def draw_map(self):
        # Draw start/goal nodes
        pygame.draw.circle(self.window, (255, 200, 0), self.start, self.node_radius, self.node_thickness)
        pygame.draw.circle(self.window, (0, 255, 0), self.goal, self.node_radius, self.node_thickness)
        # draw obstacles
        for obs in self.obstacles:
            pygame.draw.circle(self.window, (255, 255, 255), (obs.x, obs.y), obs.radius)


    def draw_path(self, path):
        pass

class RRTGraph:
    def __init__(self, start, goal, map_dimensions):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_dimensions = map_dimensions
        self.obstacles = []
        self.goal_flag = False
        self.path = []

        # Tree data structure: parent-child dictionary. keys are node coordinates, values are parent coordinates
        tree = {start: start}  # start node is its own parent

    def create_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        return self.obstacles


    def add_node(self, node):
        pass

    def remove_node(self):
        pass

    def add_edge(self, edge):
        pass

    def remove_edge(self):
        pass

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