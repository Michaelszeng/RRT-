"""
RRT* algorithm implementation and visualization.

Note: main reason it is slow is because the entire map is redrawn every loop.
However, this modification was necessary because of the nature of RRT*.

Author: Michael Zeng

pip install -r requirements.txt
"""
import pygame
import time

from Window import *
from Obstacle import *
from RRT import *
from RRT_star import *


def main():
    # Create Obstacles
    obstacles = []
    obs0 = Obstacle(620, 275, 100)
    obs0.set_trajectory({0: 0})
    obs1 = Obstacle(600, 400, 100)
    obs1.set_trajectory({0: 0})
    obs2 = Obstacle(850, 600, 80)
    obs2.set_trajectory({0: 0})
    obs3 = Obstacle(900, 650, 80)
    obs3.set_trajectory({0: 0})
    obs4 = Obstacle(250, 700, 80)
    obs4.set_trajectory({0: 0})
    obs5 = Obstacle(290, 650, 80)
    obs5.set_trajectory({0: 0})
    obstacles.append(obs0)  # 0 velocity at time 0 (and all other times
    obstacles.append(obs1)  # 0 velocity at time 0 (and all other times)
    obstacles.append(obs2)  # 0 velocity at time 0 (and all other times)
    obstacles.append(obs3)  # 0 velocity at time 0 (and all other times)
    obstacles.append(obs4)  # 0 velocity at time 0 (and all other times)
    obstacles.append(obs5)  # 0 velocity at time 0 (and all other times)

    # Initialize pygame window
    dimensions = (1200, 800)
    start = (100, 100)
    goal = (1100, 700)
    window = Window(start, goal, dimensions, obstacles)

    step_size = 40

    # SELECT ONE OF THE FOLLOWING: 
    graph = RRTGraph(start, goal, dimensions, step_size, obstacles)
    # graph = RRTStarGraph(start, goal, dimensions, step_size, obstacles)

    iteration = 0
    while iteration < 10000 and not graph.goal_flag:
        x, y, parent = graph.generate_random_next_node()  # returns valid next node
        goal_flag = graph.add_node(x, y, parent)

        window.draw_map()
        for node_id, node_value in graph.tree.items():
            if node_id == 0:
                continue
            node_x = node_value[0][0]
            node_y = node_value[0][1]
            node_parent_x = graph.tree[node_value[1]][0][0]
            node_parent_y = graph.tree[node_value[1]][0][1]
            pygame.draw.circle(window.window, (255, 255, 255), (node_x, node_y), window.node_radius, window.node_thickness)
            pygame.draw.line(window.window, (255, 0, 0), (node_x, node_y), (node_parent_x, node_parent_y), window.edge_thickness)
            pygame.display.update()
        iteration += 1

    if goal_flag:
        print("Solution found.")
        path = graph.return_path()  # list of vertices
        print(path)
        # draw path
        window.draw_map()
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