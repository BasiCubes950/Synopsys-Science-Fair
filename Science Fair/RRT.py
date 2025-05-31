import cv2
import numpy as np
import math
import random
import argparse
import os
import time

class Nodes:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

# check collision
def collision(x1, y1, x2, y2):
    color = []
    x = list(np.arange(x1, x2, (x2 - x1) / 100))
    y = [(y2 - y1) / (x2 - x1) * (xi - x1) + y1 for xi in x]

    for i in range(len(x)):
        color.append(img[int(y[i]), int(x[i])])
    return 0 in color

# check the collision with obstacle and trim
def check_collision(x1, y1, x2, y2):
    _, theta = dist_and_angle(x2, y2, x1, y1)
    x = x2 + stepSize * np.cos(theta)
    y = y2 + stepSize * np.sin(theta)

    hy, hx = img.shape
    if y < 0 or y > hy or x < 0 or x > hx:
        return x, y, False, False

    directCon = not collision(x, y, end[0], end[1])
    nodeCon = not collision(x, y, x2, y2)
    return x, y, directCon, nodeCon

# return dist and angle between new point and nearest node
def dist_and_angle(x1, y1, x2, y2):
    dist = math.hypot(x1 - x2, y1 - y2)
    angle = math.atan2(y2 - y1, x2 - x1)
    return dist, angle

# return the neaerst node
def nearest_node(x, y):
    distances = [math.hypot(n.x - x, n.y - y) for n in node_list]
    return distances.index(min(distances))

# generate a random point in the image space
def rnd_point(h, l):
    return random.randint(0, l), random.randint(0, h)

"""
RRT [Tad bit modified from nimRobotics]
https://github.com/nimRobotics/RRT
"""
def RRT(img, img2, start, end, stepSize):
    start_time = time.time()
    h, l = img.shape
    node_list[0] = Nodes(start[0], start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    cv2.circle(img2, start, 5, (0, 0, 255), thickness=3)
    cv2.circle(img2, end, 5, (0, 0, 255), thickness=3)

    i = 1
    pathFound = False
    total_distance = 0

    while not pathFound:
        nx, ny = rnd_point(h, l)
        nearest_ind = nearest_node(nx, ny)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y

        tx, ty, directCon, nodeCon = check_collision(nx, ny, nearest_x, nearest_y)

        if directCon and nodeCon:
            pathFound = True
            node_list.append(Nodes(tx, ty))
            node_list[i].parent_x = node_list[nearest_ind].parent_x + [tx]
            node_list[i].parent_y = node_list[nearest_ind].parent_y + [ty]

            cv2.circle(img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3)
            cv2.line(img2, (int(tx), int(ty)), (int(nearest_x), int(nearest_y)), (0, 255, 0), thickness=1)
            cv2.line(img2, (int(tx), int(ty)), end, (255, 0, 0), thickness=2)

            for j in range(len(node_list[i].parent_x) - 1):
                x1, y1 = node_list[i].parent_x[j], node_list[i].parent_y[j]
                x2, y2 = node_list[i].parent_x[j + 1], node_list[i].parent_y[j + 1]
                total_distance += math.hypot(x2 - x1, y2 - y1)

            break

        elif nodeCon:
            node_list.append(Nodes(tx, ty))
            node_list[i].parent_x = node_list[nearest_ind].parent_x + [tx]
            node_list[i].parent_y = node_list[nearest_ind].parent_y + [ty]
            total_distance += math.hypot(tx - nearest_x, ty - nearest_y)

            cv2.circle(img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3)
            cv2.line(img2, (int(tx), int(ty)), (int(nearest_x), int(nearest_y)), (0, 255, 0), thickness=1)
            i += 1

    computation_time = time.time() - start_time
    cv2.putText(img2, "RRT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return total_distance, computation_time


if __name__ == '__main__':
    # arguments
    parser = argparse.ArgumentParser(description='RRT Path Planning')
    parser.add_argument('-p', type=str, default='world1.png', help='Path of the maze image')
    parser.add_argument('-s', type=int, default=10, help='Step size')
    parser.add_argument('-start', type=int, nargs=2, default=None, help='Start coordinates (x y)')
    parser.add_argument('-end', type=int, nargs=2, default=None, help='End coordinates (x y)')
    args = parser.parse_args()

    img = cv2.imread(args.p, 0)
    img2 = cv2.imread(args.p)
    stepSize = args.s
    node_list = [0]
    coordinates = []

    # Set start and end
    start = tuple(args.start) if args.start else (10, img.shape[0] - 10)
    end = tuple(args.end) if args.end else (img.shape[1] - 10, 10)
    
    # record stats and save image
    with open("distance.txt", "a") as dist_file, open("time.txt", "a") as time_file:
        for _ in range(1):  # can change to run more than once [Run batches]
            total_distance, computation_time = RRT(img, img2, start, end, stepSize)
            cv2.imwrite("RRT_result.png", img2)
            dist_file.write(f"{total_distance:.2f}\n")
            time_file.write(f"{computation_time:.2f}\n")