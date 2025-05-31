import cv2
import numpy as np
import math
import random
import argparse
import time
import matplotlib.pyplot as plt

# Distance function (just distance)
def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

"""
Checks if the path between two points collides with any black pixels (obstacles)
Inspired by nimRobotics
"""
def collision(x1, y1, x2, y2):
    color = []
    if x1 == x2 and y1 == y2:
        color.append(img[int(y1), int(x1)])
    else:
        x = list(np.arange(x1, x2, (x2 - x1) / 100 if x2 != x1 else 1))
        y = list(((y2 - y1) / (x2 - x1)) * (np.array(x) - x1) + y1 if x2 != x1 else np.arange(y1, y2, (y2 - y1) / 100))
        for i in range(len(x)):
            if 0 <= int(x[i]) < img.shape[1] and 0 <= int(y[i]) < img.shape[0]:
                color.append(img[int(y[i]), int(x[i])])
    return 0 in color  # 0 = black pixel = obstacle

# Returns distance to nearest obstacle
def nearest_obstacle_distance(x, y, img):
    max_distance = 5
    for radius in range(1, max_distance):
        for angle in np.linspace(0, 2 * math.pi, 100):
            nx = int(x + radius * math.cos(angle))
            ny = int(y + radius * math.sin(angle))
            if 0 <= ny < img.shape[0] and 0 <= nx < img.shape[1] and img[ny, nx] == 0:
                return radius
    
    return max_distance

"""
Cost function
Cost = (distance to goal * collision factor) / proximity to obstacle
"""
def cost_function(node, end, img):
    x, y = node
    a = dist((x, y), end)
    b = 2 if collision(x, y, x, y) else 1 # needs to be 1 otherwise our equation falls apart [0/c is not good]
    c = nearest_obstacle_distance(x, y, img)
    return float('inf') if c == 0 else (a * b) / c

# Generates random nodes around current location
def generate_nodes_around(current, radius, count, img):
    nodes = []
    for _ in range(count):
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0, radius)
        nx = int(current[0] + distance * math.cos(angle))
        ny = int(current[1] + distance * math.sin(angle))
        if 0 <= ny < img.shape[0] and 0 <= nx < img.shape[1]:
            nodes.append((nx, ny))
    
    return nodes

# check for line of sight
def has_clear_sight(node, end):
    return not collision(node[0], node[1], end[0], end[1])

# DR-RRT
def path_planning(img, start, end, spawn_radius, node_count):
    current = start
    path = [current]
    start_time = time.time()

    while True:
        if has_clear_sight(current, end):
            path.append(end)
            break

        nodes = generate_nodes_around(current, spawn_radius, node_count, img)
        costs = [(cost_function(node, end, img), node) for node in nodes]
        costs.sort(key=lambda x: x[0])
        current = costs[0][1]
        path.append(current)

    total_time = time.time() - start_time
    total_distance = sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))
    return path, total_distance, total_time

# Plotting stuff
def plot_path(img, path, save_path="final_path.png"):
    plt.figure(figsize=(10, 10))
    plt.imshow(img, cmap="gray")

    x_coords, y_coords = zip(*path)
    plt.plot(x_coords, y_coords, marker="o", color="blue", linewidth=2, markersize=4, label="Path")
    plt.scatter(*path[0], color="green", s=100, label="Start")
    plt.scatter(*path[-1], color="red", s=100, label="End")

    plt.legend()
    plt.title("DR-RRT Path Visualization")
    plt.savefig(save_path)
    plt.close()

# MAIN
if __name__ == '__main__':
    # arguments
    parser = argparse.ArgumentParser(description='RRT Path Planning')
    parser.add_argument('-p', type=str, default='world1.png', help='Path of the maze image')
    parser.add_argument('-s', type=int, default=10, help='Step size (unused here)')
    parser.add_argument('-start', type=int, nargs=2, default=None, help='Start coordinates (x y)')
    parser.add_argument('-end', type=int, nargs=2, default=None, help='End coordinates (x y)')
    args = parser.parse_args()

    img = cv2.imread(args.p, 0)

    # Set start and end points
    start = tuple(args.start) if args.start else (10, img.shape[0] - 10)
    end = tuple(args.end) if args.end else (img.shape[1] - 10, 10)

    # record stats and save image
    with open("distance.txt", "a") as dist_file, open("time.txt", "a") as time_file:
        for _ in range(1):  # can change to run more than once [Run batches]
            path, total_distance, total_time = path_planning(img, start, end, spawn_radius=20, node_count=25)
            plot_path(img, path, "DR-RRT_result.png")
            dist_file.write(f"{total_distance:.2f}\n")
            time_file.write(f"{total_time:.2f}\n")
