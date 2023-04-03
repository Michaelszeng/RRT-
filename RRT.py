"""
Helper functions and classes for RRT* algorithm
"""
import math
import pygame

class map:
    def __init__(self, start, goal, map_dimensions):
        self.width, self.height = map_dimensions
        self.start = start
        self.goal = goal
        self.obstacles = []

    def draw_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def draw_map(self, screen):
        for obstacle in self.obstacles:
            obstacle.draw(screen)

    def draw_path(self, screen, path):
        for i in range(len(path) - 1):
            pygame.draw.line(screen, (0, 255, 0), path[i], path[i + 1], 2)

class RRTGraph:
    def __init__(self):
        pass

    def add_node(self, node):
        pass

    def remove_node(self):
        pass

    def add_edge(self, edge):
        pass

    def remove_edge(self):
        pass
