"""
My Python implementation of the A* search algorithm.

Current script uses a grid with obstacles to showcase A* framework.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np


class Node:
    """A class representing a node in the A* algorithm."""

    def __init__(self, x, y, g=0, h=0):
        """Initialize a node with coordinates (x, y) and costs g, h."""
        self.x = x
        self.y = y
        self.g = g  # actual cost
        self.h = h  # heuristic cost
        self.f = g + h
        self.parent = None

    def __eq__(self, other):
        """Check equality based on coordinates."""
        return self.x == other.x and self.y == other.y


def heuristic(node, goal):
    """Calculate Manhattan distance as heuristic between node and goal."""
    return abs(node.x - goal.x) + abs(node.y - goal.y)


def a_star(grid, start, goal):
    """Perform the A* search algorithm."""
    open_list = []  # explore
    closed_list = []  # explored

    # initializing
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.g + start_node.h

    open_list.append(start_node)

    movements = [
        (0, 1),
        (0, -1),
        (1, 0),
        (-1, 0),
    ]  # defining possible moves for my robot

    while open_list:
        current_node = min(open_list, key=lambda node: node.f)
        open_list.remove(current_node)
        closed_list.append(current_node)

        print("current position:", (current_node.x, current_node.y))

        if current_node == goal_node:
            print("found the goal node at:", (current_node.x, current_node.y))
            return reconstruct_path(current_node)

        for move in movements:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if x < 0 or x >= len(grid) or y < 0 or y >= len(grid[0]) or grid[x][y] == 1:
                continue

            neighbor = Node(x, y)

            if any(node.x == neighbor.x and node.y == neighbor.y for node in closed_list):
                continue

            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor, goal_node)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current_node

            found_in_open = False
            for node in open_list:
                if neighbor == node:
                    if neighbor.g < node.g:
                        node.g = neighbor.g
                        node.f = neighbor.f
                        node.parent = current_node
                    found_in_open = True
                    break

            if not found_in_open:
                open_list.append(neighbor)

    raise ValueError("couldn't find path")


def reconstruct_path(node):
    """Reconstruct the path from the goal node to the start node."""
    path = []
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


if __name__ == "__main__":
    USE_RANDOM_GRID = True
    if USE_RANDOM_GRID:
        PROBABILITY_OF_A_CELL_BEING_AN_OBSTACLE = 0.2
        GRID_SIZE = 100
        rng = np.random.default_rng()
        THRESHOLD_FOR_GRID_GENERATION = PROBABILITY_OF_A_CELL_BEING_AN_OBSTACLE * 100
        grid = rng.integers(low=0, high=101, size=(GRID_SIZE, GRID_SIZE)) <= THRESHOLD_FOR_GRID_GENERATION
        print(grid)
        start = (0, 0)
        goal = (GRID_SIZE - 1, GRID_SIZE - 1)
        grid[start] = 0
        grid[goal] = 0
    else:
        grid = [
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0],
            [1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1],
            [0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0],
            [1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0],
            [1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0],
            [1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0],
            [0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        ]
        start = (0, 0)
        goal = (19, 19)

    try:
        path = a_star(grid, start, goal)
        print("found the shortest path:", path)
    except ValueError as e:
        print(e)

    plt.figure()
    plt.spy(np.array(grid).T)
    for waypoint in path:
        hwaypoint = plt.scatter(waypoint[0], waypoint[1], 10, color="r")
    hstart = plt.scatter(start[0], start[1], 50, color="b")
    hgoal = plt.scatter(goal[0], goal[1], 50, color="g")
    plt.legend([hstart, hgoal, hwaypoint], ["Start", "Goal", "Waypoint"], bbox_to_anchor=(1.35, 0.5))
    plt.tight_layout()
    plt.show()
