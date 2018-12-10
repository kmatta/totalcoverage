'''
RBE550 Fall 2018 Total Coverage Team Project
WPI MS Robotics engineering
Ryan Ferrin, Karthick Dhavamani, Krishna Matta
2018/Dec/09
1 of 3 algorithms tested for complete Coverage

This is an Implementation of Complete Coverage D* algorithm described by
Marija Dakulovic ∗ Sanja Horvatic ∗ Ivan Petrovic. 2011

implementation by Ryan Ferrin 2018

The code simulates new and dynamic obstacles by loading in variations of the first map. to run select which level to solve
and un-comment the group of 4 images. and comment out the other images
'''

from PIL import Image
import numpy
import queue as Q
import time

# colors used to draw images
white = (255, 255, 255)
black = (0, 0, 0)
blue = (0, 0, 255)
green = (0, 255, 0)
red = (255, 0, 0)
purple = (85, 26, 139)
yellow = (255, 255, 100)

# map difficultly
# easiest
# pic1 = "bedroom.gif"
# pic2 = "bedroom_2.gif"
# pic3 = "bedroom_3.gif"
# pic4 = "bedroom_4.gif"

# easy - few obstacles This set requires 2 more images to run
# pic1 = "test3.gif"
# pic2 = "test3_2.gif"

# # easy - many obstacles
pic1 = "Store.gif"
pic2 = "Store2.gif"
pic3 = "Store3.gif"
pic4 = "Store4.gif"

#Medium
# pic1 = "room.gif"
# pic2 = "room_2.gif"
# pic3 = "room_3.gif"
# pic4 = "room_4.gif"

# very Hard
# pic1 = "test1.gif"


# convert images to numeric grid
def scan(area):
    obs_loc = []
    cfree = []
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
                cfree.append((i, j))
            elif value == black:
                grid[i, j] = 1
                obs_loc.append((i, j))
    print("number of obstacle nodes" + str(len(obs_loc)))
    print("number of free nodes" + str(len(cfree)))

    return grid, columns, rows, im

# iterate the map to be used
def mapcount(count):

    if count == 2:
        output = scan(pic2)
    elif count == 3:
        output = scan(pic3)
    elif count == 4:
        output = scan(pic4)
    else:
        output = scan(pic1)
    return output


# find repeated nodes
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
    print("number of repeated nodes" + str(len(revisited)))
    return revisited

# cost used by CCD*
def cost(x, y, end):
    h = (abs(x - end[0]) + abs(y - end[1])) * -1
    return h

# cost used by D*
def bt_cost(x, y, goal):
    k = (abs(x - goal[0]) + abs(y - goal[1]))
    return k

# classify neighbor nodes for CCD*
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
        if (x, y) in front:
            front.remove((x, y))
    # Check if path clear
    elif grid[x, y] == 0:
        h = cost(x, y, end)
        neighbors.put((h, (x, y)))
        if (x, y) not in front:
            front.append((x, y))
        return
    else:
        print(grid[x, y])

# classify neighbor nodes of D*
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

# neighbor search for D*
def bt_explore(bt_pos, goal, bt_path, bt_next, grid, columns, rows):

    bt_neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    bt_sort(bt_pos[0] - 1, bt_pos[1], goal, bt_path, grid, bt_neighbors, columns, rows)
    # look up
    bt_sort(bt_pos[0], bt_pos[1] - 1, goal, bt_path, grid, bt_neighbors, columns, rows)
    # look right
    bt_sort(bt_pos[0] + 1, bt_pos[1], goal, bt_path, grid, bt_neighbors, columns, rows)
    # look down
    bt_sort(bt_pos[0], bt_pos[1] + 1, goal, bt_path, grid, bt_neighbors, columns, rows)
    if len(bt_neighbors.queue) == 0:
        print("lost")
        # return
    bt_first = bt_neighbors.get()
    bt_next.append(bt_first[1])

# find closest "dirty" or unexplored node for D* search
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

# preform D* search if no available neighbors in CCD*
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
        bt_explore(bt_pos, goal, bt_path, bt_next, grid, columns, rows)


# explore neighbors for CCD*
def explore(current_pos, grid, columns, rows, path, front, prox):

    neighbors = Q.PriorityQueue()  # priority q for planning algorithm
    # look left
    sort(current_pos[0] - 1, current_pos[1], end, path, grid, neighbors, columns, rows, front)
    # look up
    sort(current_pos[0], current_pos[1] -1, end, path, grid, neighbors, columns, rows, front)
    # look right
    sort(current_pos[0] + 1, current_pos[1], end, path, grid, neighbors, columns, rows, front)
    # look down
    sort(current_pos[0], current_pos[1] + 1, end, path, grid, neighbors, columns, rows, front)

    if len(neighbors.queue) == 0:
        backtrack(current_pos, grid, columns, rows, front, path, prox)
        return
    first = neighbors.get()
    if first[1] == end and len(front) > 1:
        backtrack(current_pos, grid, columns, rows, front, path, prox)
        return
    prox.append(first[1])

# produce image of search/path execution
def visualize_search(im, path, revisited):
    pixel_access = im.load()

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = blue

    # draw cleaned pixels
    for pixel in clean:
        pixel_access[pixel[0], pixel[1]] = yellow

    # draw revisited pixels
    for pixel in revisited:
        pixel_access[pixel[0], pixel[1]] = purple

    # draw start and end pixels
    pixel_access[start[0], start[1]] = green
    pixel_access[end[0], end[1]] = red

    # display and (maybe) save results
    out = im.resize((500, 500))
    out.show()
    out.save("out.png")

# run the CCD* planner
def ccd_star_plan(start, end, front, grid, columns, rows, im):
    prox = []  # a list of all the places left to explore
    path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal


    front.append(start)
    prox.append(start)

    # Get start time
    plan_runstart = time.time()

    while len(front) != 0:

        current_pos = prox.pop()
        if current_pos in front:
            front.remove(current_pos)
        path.append(current_pos)
        if current_pos == end:
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

# identify nodes that are still dirty before re-planning
def nob_front(clean, dirty):
    front= []
    n = len(clean)
    for i in range(len(dirty) - len(clean)):
        front.append(dirty[n])
        n += 1
    return front


#follow planned path
# Get start time

runstart = time.time()

start = (0, 1)
end = (0, 0)

front = []  # a list of where we haven't tried to go
clean = [] # a list of where we've been

count = 1
output1 = mapcount(count)
grid1 = output1[0]
columns = output1[1]
rows = output1[2]
im1 = output1[3]

dirty = ccd_star_plan(start, end, front, grid1, columns, rows, im1)
dirty1 = dirty

count = 2
output2 = mapcount(count)
grid = output2[0]
im = output2[3]
spot = 0
i=0

while spot != end:
    spot = dirty[i]
    clean.append(spot)
    if spot == end:
         break
    #look ahead
    ahead = dirty[i + 1]
    if grid[ahead[0], ahead[1]] == 1:
        # front = nob_front(clean, dirty1)
        print(len(front))
        dirty = ccd_star_plan(spot, end, front, grid, columns, rows, im)
        count = count + 1
        output = mapcount(count)
        grid = output[0]
        im = output[3]
        i = 0
    else:
        i = i + 1

runstop = time.time()
print('Final Path Length ' + str(len(clean)))
print('TOTAL EXECUTION TIME FOR Planning: ' + str(runstop - runstart))

dupe = overlapped(clean)
visualize_search(im1, dirty1, dupe)
