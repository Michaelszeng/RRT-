"""
RRT* algorithm implementation and visualization

Author: Michael Zeng

pip install -r requirements.txt
"""
import pygame
import time
from RRT import *


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

    # Initialize RRT Graph
    graph = RRTGraph(start, goal, dimensions, 20, obstacles)

    iteration = 0
    while iteration < 10000 and not graph.goal_flag:
        x, y, parent = graph.generate_random_next_node()  # returns valid next node
        goal_flag = graph.add_node(x, y, parent)
        pygame.draw.circle(window.window, (255, 255, 255), (x, y), window.node_radius, window.node_thickness)
        pygame.draw.line(window.window, (255, 0, 0), (x, y), (graph.tree[parent][0][0], graph.tree[parent][0][1]), window.edge_thickness)
        pygame.display.update()
        iteration += 1

    if goal_flag:
        print("Solution found.")
        path = graph.return_path()
        print(path)
        # draw path
        for i, vtx in enumerate(path[:-1]):
            pygame.draw.circle(window.window, (0, 255, 0), (vtx[0], vtx[1]), window.node_radius, window.node_thickness)
            pygame.draw.line(window.window, (0, 200, 0), (vtx[0], vtx[1]), (path[i+1][0], path[i+1][1]), window.edge_thickness)
    else:
        print("No solution found.")

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)  # freezes window until keypress



if __name__ == "__main__":
    print()
    main()