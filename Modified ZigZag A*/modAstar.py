import sys, os
from PIL import Image
import copy
from Queue import PriorityQueue
import heapq
import pdb
import numpy as np
import math
import time
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
left_boundary_wp = []
right_boundary_wp = []
left_obstacle_wp = []
right_obstacle_wp = []
obstacle_wp = []
a_star_path = []
traversed = []
explored = {}
frontier = {}

NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

# To invert the image if necessary: In general 0 is obstacle 1 is free space
def invert(im,size):
    for i in range(size[0]):
        for j in range(size[1]):
            im[i,j] = 1 - im[i,j]
    return im

# Find boundary Waypoints
def find_bwp(im, size):

    # To find left boundary waypoints
    for j in range(size[1]):
        for i in range(2):
            if img[i,j] == 0:
                left_boundary_wp.append([i,j])
                break

    # To find right boundary waypoints
    for j in range(size[1]):
        for i in range(size[0]-1,size[0]-3,-1):
            if img[i,j] == 0:
                right_boundary_wp.append([i,j])
                break

    # Assert if the map has left and right boundaries, if not assign
    for j in range(size[1]):

        found_left_boundary = False
        found_right_boundary = False

        for i in range(size[0]):

            if [i,j] in left_boundary_wp:
                found_left_boundary = True
            if [i,j] in right_boundary_wp:
                found_right_boundary = True


        if not found_left_boundary:
            left_boundary_wp.append([0,j])
        if not found_right_boundary:
            right_boundary_wp.append([size[0]-1,j])

# Find Obstacle Waypoints
def find_owp(im, size):

    # To find left obstacle waypoints
    for j in range(size[1]):
        for i in range(size[0]-1):
            if img[i,j] == 1 and img[i+1,j] == 0:
                if [i,j] not in left_boundary_wp:
                    left_obstacle_wp.append([i,j])

    # To find right obstacle waypoints
    for j in range(size[1]):
        for i in reversed(range(size[0])):
            if i>0 and img[i,j] == 1 and img[i-1,j] == 0:
                if [i,j] not in right_boundary_wp:
                    right_obstacle_wp.append([i,j])

# Calculate Manhattan Cost for A Star
def cost(a, b):
	(x1, y1) = a
	(x2, y2) = b
	h = abs(x2 - x1) + abs(y2 - y1)
	return h

# Expand the nodes for A Star
def expand(map, size, node):
	x = node[0]
	y = node[1]
	results = []

	x_max = size[0]
	y_max = size[1]

	if (x+1 < x_max) and (map[x+1,y] != 0):
		results.append((x+1,y))

	if (y+1 < y_max) and (map[x,y+1] != 0):
		results.append((x,y+1))

	if (y>=1) and (map[x,y-1] != 0):
		results.append((x,y-1))

	if (x>=1) and (map[x-1,y] != 0):
		results.append((x-1,y))

	return results

# Main A Star Algorithm
def A_star(map, size, start, goal):
	front = PriorityQueue(0)
	front.put((0, start))
	parent = {}
	explored = {}

	parent[start] = None
	explored[start] = 0
	s = 0
	while front.qsize():
		current_node = front.get()[1]

		if current_node == goal:
			s = front.get()[0]
			break

		for node in expand(map, size, current_node):
			new_cost = explored[current_node] + 1
			if node not in explored or new_cost < explored[node] :
				explored[node] = new_cost
				total_cost = new_cost + cost(goal, node)
				front.put((total_cost, node))
				parent[node] = current_node

	path = []
	while current_node != start:
		path.append(current_node)
		current_node = parent[current_node]
	path.append(start)
	path.reverse()

	frontier = {}
	for f in front.queue:
		frontier[f[1]] = f[0]

	return path

# Main Body of complete coverage algorithm
def complete_coverage(img, size):

    global a_star_path

    im = img.load()
    current = [0,0]
    left2right = True
    pending_left = False
    pending_left_points = []
    pending_right = False
    pending_right_points = []

    # Store the intermediate nodes as traversed
    def traverse(start, end):
        if start[0]<end[0]:
            for i in range(start[0],end[0]+1):
                traversed.append([i,start[1]])
                if im[i,start[1]] == 0:
                    pdb.set_trace() # Stop if any obstacle spaces are traversed
        else:
            for i in range(end[0],start[0]+1):
                traversed.append([i,start[1]])
                if im[i,start[1]] == 0:
                    pdb.set_trace() # Stop if any obstacle spaces are traversed

    j = 0
    while j < size[1]:

        current[1] = j
        path_planned = False

        # If current is in obstacle region, move to non obstacle region
        if im[tuple(current)] == 0:
            if left2right:
                for i in range(current[0],size[0]):
                    if im[i,j] == 1:
                        current = [i,j]
                        break
            else:
                for i in range(current[0],-1,-1):
                    if im[i,j] == 1:
                        current = [i,j]
                        break

        # ZigZag traversal either left2right or right2left
        if left2right:

            found_left_obstacle = False
            left_boundary = []

            for i in range(current[0]+1,size[0]):

                if [i,j] in left_obstacle_wp or [i,j] in right_boundary_wp:
                    if [i,j] in left_obstacle_wp:
                        found_left_obstacle = True
                    left_boundary.append([i,j])
                    break

            if not found_left_obstacle and not pending_left:
                start = current
                end = [size[0]-1,j]
            elif found_left_obstacle:
                if not pending_left:
                    start = current
                    end = left_boundary[0]
                    for it in range(end[0]+1,size[0]):
                        if [it,j] in right_obstacle_wp and [it,j] not in traversed:
                            pending_left = True
                            pending_left_points.append([it,j])
                            break
                else:
                    start = current
                    if left_boundary[0][0] < pending_left_points[0][0]:
                        end = left_boundary[0]
                    else:
                        end = pending_left_points.pop()
                        a_star_path = A_star(im,size,tuple(current),tuple(end))
                        for item in a_star_path:
                            if [item[0],item[1]] not in traversed:
                                traversed.append([item[0],item[1]])
                        path_planned = True
                        if not pending_left_points:
                            pending_left = False
                        j = end[1] - 1
                        left2right = not left2right

            else:
                end = pending_left_points.pop()
                a_star_path = A_star(im,size,tuple(current),tuple(end))
                for item in a_star_path:
                    if [item[0],item[1]] not in traversed:
                        traversed.append([item[0],item[1]])
                path_planned = True
                if not pending_left_points:
                    pending_left = False
                j = end[1] - 1
                left2right = not left2right

        else:
            found_right_obstacle = False
            right_boundary = []

            for i in range(current[0]-1,-1,-1):

                if [i,j] in right_obstacle_wp or [i,j] in left_boundary_wp:
                    if [i,j] in right_obstacle_wp:
                        found_right_obstacle = True
                    right_boundary.append([i,j])
                    break

            if not found_right_obstacle and not pending_right:
                start = current
                end = [0,j]
            elif found_right_obstacle:
                if not pending_right:
                    start = current
                    end = right_boundary[0]
                    for it in range(end[0]-1,-1,-1):
                        if [it,j] in left_obstacle_wp and [it,j] not in traversed:
                            pending_right = True
                            pending_right_points.append([it,j])
                            break
                else:
                    start = current
                    if right_boundary[0][0] > pending_right_points[0][0]:
                        end = right_boundary[0]
                    else:
                        end = pending_right_points.pop()
                        a_star_path = A_star(im,size,tuple(current),tuple(end))
                        for item in a_star_path:
                            if [item[0],item[1]] not in traversed:
                                traversed.append([item[0],item[1]])
                        path_planned = True
                        if not pending_right_points:
                            pending_right = False
                        j = end[1] - 1
                        left2right = not left2right

            else:
                end = pending_right_points.pop()
                a_star_path = A_star(im,size,tuple(current),tuple(end))
                for item in a_star_path:
                    if [item[0],item[1]] not in traversed:
                        traversed.append([item[0],item[1]])
                path_planned = True
                if not pending_right_points:
                    pending_right = False
                j = end[1] - 1
                left2right = not left2right

        if not path_planned:
            traverse(start, end)
        current[0] = end[0]
        left2right = not left2right
        j = j + 1

# Plot the map
def plot_map(im,filename = "do_not_save.png"):
    im = im.convert("RGB")
    pixel_access = im.load()

    for pixel in left_boundary_wp:
        pixel_access[pixel[0], pixel[1]] = NEON_GREEN

    for pixel in right_boundary_wp:
        pixel_access[pixel[0], pixel[1]] = NEON_GREEN

    for pixel in left_obstacle_wp:
        pixel_access[pixel[0], pixel[1]] = NEON_GREEN

    for pixel in right_obstacle_wp:
        pixel_access[pixel[0], pixel[1]] = NEON_GREEN

    for pixel in traversed:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    plt.imshow(im)
    plt.show(block = False)
    fig.canvas.draw()

    # To save the file
    if(filename != "do_not_save.png"):
        im.save(filename)
    im.close()

if __name__ == "__main__":

    assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)

    # Perform search on given image
    im = Image.open("Store.gif")
    size = im.size
    img = im.load()
    img = invert(img,size) # If image is inverted use this line

    # Count the total free space configurations
    free_space = 0
    for i in range(size[0]):
        for j in range(size[1]):
            if img[i,j] == 1:
                free_space = free_space + 1

    find_bwp(img, size) # Find boundary waypoints
    find_owp(img, size) # Find obstacle waypoints
    tic = time.time()
    complete_coverage(im, size) # Main Algorithm definition
    toc = time.time()
    print "Time Taken: {0} secs".format(toc-tic)
    print "Percentage Coverage: {0:.2f} %".format(len(traversed)*100/free_space)
    pdb.set_trace()
    plot_map(im,"out.png") # Save the output file in the current folder
