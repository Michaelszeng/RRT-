"""
RRT* algorithm implementation and visualization

Author: Michael Zeng

pip install -r requirements.txt
"""
import pygame
from RRT import *


step_size = 1


def main():
    # Create Obstacles
    obstacles = []
    obs1 = Obstacle(600, 400, 100)
    obs1.set_trajectory({0: 0})
    obs2 = Obstacle(850, 600, 80)
    obs2.set_trajectory({0: 0})
    obstacles.append(obs1)  # 0 velocity at time 0 (and all other times)
    obstacles.append(obs2)  # 0 velocity at time 0 (and all other times)

    # Initialize pygame window
    dimensions = (1200, 800)
    start = (100, 100)
    goal = (1100, 700)
    window = Window(start, goal, dimensions, obstacles)
    window.draw_map()

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)  # freezes window until keypress



if __name__ == "__main__":
    print()
    main()