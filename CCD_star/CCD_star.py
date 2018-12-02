from PIL import Image
# import pygame
import numpy
import queue as Q
import time

white = (255, 255, 255)
black = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
PURPLE = (85, 26, 139)
YELLOW = (255, 255, 100)

# map deficullty
# easiest
# pic1 = "test.gif"
# pic2 = "test_2.gif"

# easy - few obstacles
# pic1 = "test3.gif"
# pic2 = "test3_2.gif"

# easy - many obstacles
# pic1 = "Store.gif"
# pic2 = "Store2.gif"

# Medium
pic1 = "test2.gif"
pic2 = "test2_2.gif"

# very Hard
# pic1 = "test1.gif"


def scan(area):
    im = Image.open(area).convert("RGB")
    size = im.size
    columns = size[0]
    rows = size[1]
    grid = numpy.zeros((rows, columns))

    for i in range(columns):
        for j in range(rows):
            value = im.getpixel((i, j))
            if value == white:
                grid[i, j] = 0
            elif value == black:
                grid[i, j] = 1

    return grid, columns, rows, im


def overlapped(path):
    seen = {}
    revisited = []

    for x in path:
        if x not in seen:
            seen[x] = 1
        else:
            if seen[x] == 1:
                revisited.append(x)
            seen[x] += 1
    return revisited

def cost(x, y, end):
    h = (abs(x - end[0]) + abs(y - end[1])) * -1
    return h


def bt_cost(x, y, goal):
    k = (abs(x - goal[0]) + abs(y - goal[1]))
    return k


def sort(x, y, end, path, grid, neighbors, columns, rows, front):

    # Check for boundary
    if x < 0 or x > columns - 1 or y < 0 or y > rows - 1:
        return
    # Check if the node has already been explored
    elif (x, y) in path:
        return
    elif (x, y) in clean:
        return
    # check for obstacle
    elif grid[x, y] == 1:
        return
    # Check if path clear
    elif grid[x, y] == 0:
        h = cost(x, y, end)
        neighbors.put((h, (x, y)))
        if (x, y) not in front:
            front.append((x, y))
        return
    else:
        print(grid[x, y])


def bt_sort(x, y, goal, bt_path, grid,  neighbors, columns, rows):
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


def bt_explore(bt_pos, goal, bt_path, bt_next, grid, columns, rows):

    bt_neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    bt_sort(bt_pos[0], bt_pos[1] - 1, goal, bt_path, grid, bt_neighbors, columns, rows)
    # look up
    bt_sort(bt_pos[0] - 1, bt_pos[1], goal, bt_path, grid, bt_neighbors, columns, rows)
    # look right
    bt_sort(bt_pos[0], bt_pos[1] + 1, goal, bt_path, grid, bt_neighbors, columns, rows)
    # look down
    bt_sort(bt_pos[0] + 1, bt_pos[1], goal, bt_path, grid, bt_neighbors, columns, rows)
    bt_first = bt_neighbors.get()
    bt_next.append(bt_first[1])


def nearest_front(current_pos, front):
    near = 1.0e12
    if len(front) <= 1:
        goal = end
        return goal
    else:
        for n in range(len(front)):
            dist = (abs(current_pos[0] - front[n][0]) + abs(current_pos[0] - front[n][0]))
            if dist < near and front[n] != end:
                near = dist
                goal = front[n]
    return goal


def backtrack(current_pos, grid, columns, rows, front, path, prox):
    bt_next = []
    bt_path = []
    goal = nearest_front(current_pos, front)
    bt_explore(current_pos, goal, bt_path, bt_next, grid, columns, rows)

    while len(bt_next) != 0:

        bt_pos = bt_next.pop()
        path.append(bt_pos)
        if bt_pos == goal:
            prox.append(bt_pos)
            return
        bt_path.append(bt_pos)
        # revisited.append(bt_pos)
        bt_explore(bt_pos, goal, bt_path, bt_next, grid, columns, rows)



def explore(current_pos, grid, columns, rows, path, front, prox):

    neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    sort(current_pos[0], current_pos[1] - 1, end, path, grid, neighbors, columns, rows, front)
    # look up
    sort(current_pos[0] - 1, current_pos[1], end, path, grid, neighbors, columns, rows, front)
    # look right
    sort(current_pos[0], current_pos[1] + 1, end, path, grid, neighbors, columns, rows, front)
    # look down
    sort(current_pos[0] + 1, current_pos[1], end, path, grid, neighbors, columns, rows, front)

    if len(neighbors.queue) == 0:
        # print("stuck")
        # print(path)
        # visualize_search()
        backtrack(current_pos, grid, columns, rows, front, path, prox)
        return
    first = neighbors.get()
    if first[1] == end and len(front) > 1:
        backtrack(current_pos, grid, columns, rows, front, path, prox)
        return
    prox.append(first[1])


def visualize_search(im, path, revisited):
    pixel_access = im.load()

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = BLUE

    # draw cleaned pixels
    for pixel in clean:
        pixel_access[pixel[0], pixel[1]] = YELLOW

    # draw revisited pixels
    for pixel in revisited:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = GREEN
    pixel_access[end[0], end[1]] = RED

    # display and (maybe) save results
    im.show()
    im.save("out.png")


def ccd_star_plan(start, end, prox, grid, columns, rows, im):
    front = []  # a list of all the places left to explore
    path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal


    front.append(start)
    prox.append(start)

    # Get start time
    plan_runstart = time.time()

    while len(front) != 0:

        current_pos = prox.pop()
        front.remove(current_pos)
        path.append(current_pos)
        if current_pos == end:
            print(front)
            break
        explore(current_pos, grid, columns, rows, path, front, prox)

    # Get end time
    plan_runstop = time.time()
    print('Path Length ' + str(len(path)))
    print('TOTAL EXECUTION TIME FOR Planning: ' + str(plan_runstop - plan_runstart))

    revisited = overlapped(path)
    visualize_search(im, path, revisited)
    # print(path)
    return path


def nob_prox(clean, dirty):
    prox = []
    n = len(clean)
    for i in range(len(dirty) - len(clean)):
        prox.append(dirty[n])
        n = n + 1
    return prox


# Get start time
runstart = time.time()

start = (0, 1)
end = (0, 0)

prox = []  # a list of where we haven't tried to go
clean = [] # a list of where we've been
# dirty = []

output1 = scan(pic1)
grid1 = output1[0]
columns = output1[1]
rows = output1[2]
im1 = output1[3]

dirty = ccd_star_plan(start, end, prox, grid1, columns, rows, im1)

output2 = scan(pic2)
grid2 = output2[0]
columns2 = output2[1]
rows2 = output2[2]
im2 = output2[3]

spot = 0
i=0

while spot != end:
    spot = dirty[i]
    clean.append(spot)
    if  spot == end:
         break
    #look ahead
    ahead = dirty[i + 1]
    if grid2[ahead[0], ahead[1]] == 1:
        prox = nob_prox(clean, dirty)
        # nob_goal = dirty[i + 2]
        # nob_d_star(dirty[i], nob_goal, i, dirty, grid2, columns2, rows2)
        dirty = ccd_star_plan(spot, end, prox, grid2, columns, rows, im2)
        i = 0
    else:
        i = i + 1

runstop = time.time()
print('Final Path Length ' + str(len(clean)))
print('TOTAL EXECUTION TIME FOR Planning: ' + str(runstop - runstart))

dupe = overlapped(clean)
visualize_search(im2, dirty, dupe)
