"""
RRT* algorithm implementation and visualization

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
    dimensions = (1200, 600)
    start = (100, 100)
    goal = (1100, 450)
    window = Window(start, goal, dimensions, obstacles)
    window.draw_map()

    step_size = 20

    # SELECT ONE OF THE FOLLOWING: 
    RRT = False
    RRT_Star = True
    RRT_Star_FN = False

    if RRT:
        graph = RRTGraph(start, goal, dimensions, step_size, obstacles)
    elif RRT_Star:
        graph = RRTStarGraph(start, goal, dimensions, step_size, obstacles)
    elif RRT_Star_FN:
        pass

    iteration = 0
    while iteration < 1000:
        print("Iteration: ", iteration)
        time.sleep(0.1)
        x, y, nearest_node = graph.generate_random_next_node()  # returns valid next node
        new_node, goal_flag, rewired_edges = graph.add_node(x, y, nearest_node)

        pygame.draw.circle(window.window, (255, 255, 255), (x, y), window.node_radius, window.node_thickness)
        
        # Only draw edge between new node and nearest_node if there was no successful rewiring of the new node
        if (new_node, nearest_node) not in rewired_edges:
            pygame.draw.line(window.window, (255, 0, 0), (x, y), (graph.tree[nearest_node][0][0], graph.tree[nearest_node][0][1]), window.edge_thickness)

        for delete_edge, add_edge in rewired_edges.items():
            # Drawing deleted edge
            delete_edge_x1 = graph.tree[delete_edge[0]][0][0]
            delete_edge_y1 = graph.tree[delete_edge[0]][0][1]
            delete_edge_x2 = graph.tree[delete_edge[1]][0][0]
            delete_edge_y2 = graph.tree[delete_edge[1]][0][1]
            # draw a black line of the edge to delete
            pygame.draw.line(window.window, (0, 0, 0), (delete_edge_x1, delete_edge_y1), (delete_edge_x2, delete_edge_y2), window.edge_thickness)

            # Drawing added edge
            add_edge_x1 = graph.tree[add_edge[0]][0][0]
            add_edge_y1 = graph.tree[add_edge[0]][0][1]
            add_edge_x2 = graph.tree[add_edge[1]][0][0]
            add_edge_y2 = graph.tree[add_edge[1]][0][1]
            # draw the new edge that replaced it
            pygame.draw.line(window.window, (255, 0, 0), (add_edge_x1, add_edge_y1), (add_edge_x2, add_edge_y2), window.edge_thickness)

        pygame.display.update()

        if goal_flag:
            if RRT:  # For RRT, end once goal is reached
                break
            else:  # For RRT*, continue until max iterations reached since we can continue to optimize the path.
                path = graph.return_path()
                # draw path
                for i, vtx in enumerate(path[:-1]):
                    pygame.draw.circle(window.window, (0, 255, 0), (vtx[0], vtx[1]), window.node_radius, window.node_thickness)
                    pygame.draw.line(window.window, (0, 200, 0), (vtx[0], vtx[1]), (path[i+1][0], path[i+1][1]), window.edge_thickness)

        iteration += 1

    if goal_flag:
        print("Solution found.")
        path = graph.return_path()
        print(path)
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