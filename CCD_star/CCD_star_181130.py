from PIL import Image
import numpy
import queue as Q
import time

white = (255, 255, 255)
black = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
PURPLE = (85, 26, 139)

# area = "test.gif"
area = "test1.gif"
# area = "test2.gif"
# area = "test3.gif"
# area = "Store.gif"

im = Image.open(area).convert("RGB")
size = im.size
columns = size[0]
rows = size[1]
grid = numpy.zeros((rows, columns))

for i in range(columns):
    for j in range(rows):
        value = im.getpixel((i,j))
        if value == white:
            grid[i, j] = 0
        elif value == black:
            grid[i, j] = 1

def cost(x, y, end):
    h = (abs(x - end[0]) + abs(y - end[1])) * -1
    return h

def bt_cost(x, y, goal):
    k = (abs(x - goal[0]) + abs(y - goal[1]))
    return k

def sort(x, y, end, path, grid, neighbors):

    # Check for boundary
    if x < 0 or x > columns - 1 or y < 0 or y > rows - 1:
        return
    # Check if the node has already been explored
    elif (x, y) in path:
        return
    # check for obstacle
    elif grid[x, y] == 1:
        return
    # Check if path clear
    elif grid[x, y] == 0:
        h = cost(x, y, end)
        neighbors.put((h, (x, y)))
        if (x, y) not in open:
            open.append((x, y))
        return
    else:
        print(grid[x, y])

def bt_sort(x, y, goal, bt_path, grid,  neighbors):
    # Check for boundary
    if x < 0 or x > columns - 1 or y < 0 or y > rows - 1:
        return
    # Check if the node has already been explored
    elif (x, y) in bt_path:
        return
    # check for obstacle
    elif grid[x, y] == 1:
        return
    # Check if path clear
    elif grid[x, y] == 0:
        k = bt_cost(x, y, goal)
        neighbors.put((k, (x, y)))
    else:
        print(grid[x, y])

def bt_explore(bt_pos, goal, bt_path, bt_next):

    bt_neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    bt_sort(bt_pos[0], bt_pos[1] - 1, goal, bt_path, grid, bt_neighbors)
    # look up
    bt_sort(bt_pos[0] - 1, bt_pos[1], goal, bt_path, grid, bt_neighbors)
    # look right
    bt_sort(bt_pos[0], bt_pos[1] + 1, goal, bt_path, grid, bt_neighbors)
    # look down
    bt_sort(bt_pos[0] + 1, bt_pos[1], goal, bt_path, grid, bt_neighbors)
    bt_first = bt_neighbors.get()
    bt_next.append(bt_first[1])

def backtrack(current_pos):

    bt_next = []
    bt_path = []

    near = 1.0e12
    for n in range(len(open)):
        dist = (abs(current_pos[0] - open[n][0]) + abs(current_pos[0] - open[n][0]))
        if dist < near:
            near = dist
            goal = open[n]


    bt_explore(current_pos, goal, bt_path, bt_next)

    while len(bt_next) != 0:

        bt_pos = bt_next.pop()
        path.append(bt_pos)
        if bt_pos == goal:
            next.append(bt_pos)
            return
        bt_path.append(bt_pos)
        revisited.append(bt_pos)
        bt_explore(bt_pos, goal, bt_path, bt_next)


def explore(current_pos):

    neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    sort(current_pos[0], current_pos[1] - 1, end, path, grid, neighbors)
    # look up
    sort(current_pos[0] - 1, current_pos[1], end, path, grid, neighbors)
    # look right
    sort(current_pos[0], current_pos[1] + 1, end, path, grid, neighbors)
    # look down
    sort(current_pos[0] + 1, current_pos[1], end, path, grid, neighbors)

    if len(neighbors.queue) == 0:
        # print("stuck")
        # print(path)
        # visualize_search()
        backtrack(current_pos)
        return
    first = neighbors.get()
    next.append(first[1])

def visualize_search():
    pixel_access = im.load()

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = BLUE

    # draw path pixels
    for pixel in revisited:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = GREEN
    pixel_access[end[0], end[1]] = RED

    # display and (maybe) save results
    im.show()
    im.save("out.png")

start = (0, 1)
end = (0, 0)

open = []  # a list of all the places left to explore
next = []  # a list of where we go from here
path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
revisited = [] # a list of places visited more than once.

open.append(start)
next.append(start)

# Get start time
runStart = time.time()

while len(open) != 0:

    current_pos = next.pop()
    open.remove(current_pos)
    path.append(current_pos)
    if current_pos == end:
        break
    explore(current_pos)

# Get end time
runStop = time.time()
print('Path Length ' + str(len(path)))
print('TOTAL EXECUTION TIME FOR SOLUTION: ' + str(runStop - runStart))

visualize_search()
# print(path)
