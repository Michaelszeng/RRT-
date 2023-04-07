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
        self.node_radius = 3
        self.node_thickness = 0
        self.edge_thickness = 1

    def draw_map(self):
        # Draw start/goal nodes
        pygame.draw.circle(self.window, (255, 200, 0), self.start, self.node_radius, self.node_thickness)
        pygame.draw.circle(self.window, (0, 255, 0), self.goal, self.node_radius, self.node_thickness)
        # draw obstacles
        for obs in self.obstacles:
            pygame.draw.circle(self.window, (255, 255, 255), (obs.x, obs.y), obs.radius)