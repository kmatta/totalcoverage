'''
RBE550 Fall 2018 Total Coverage Team Project
WPI MS Robotics engineering
Ryan Ferrin, Karthick Dhavamani, Krishna Matta
2018/Dec/09
1 of 3 algorithms tested for complete Coverage

This is an Implementation of Complete Coverage D* algorithm described by
Marija Dakulovic ∗ Sanja Horvatic ∗ Ivan Petrovic. 2011

implementation by Ryan Ferrin 2018

the code simulates new and dynamic obstacles by loading in variations of the first map. to run select which level to solve
and un-comment the group of 4 images. and comment out the other images also included are command used for the turtle to
draw the  rooms an plat the paths
'''

from PIL import Image
import turtle
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

# map deficullty
# easiest
pic1 = "bedroom.gif"
pic2 = "bedroom_2.gif"
pic3 = "bedroom_3.gif"
pic4 = "bedroom_4.gif"
scl_fctr = 57
bot_size = (3, 2)
redraw = 1

# easy - few obstacles
# pic1 = "test3.gif"
# pic2 = "test3_2.gif"

# # easy - many obstacles
# pic1 = "Store.gif"
# pic2 = "Store2.gif"
# pic3 = "Store3.gif"
# pic4 = "Store4.gif"
# scl_fctr = 7
# bot_size = (.5, .5)
# redraw = 3

#Medium
# pic1 = "room.gif"
# pic2 = "room_2.gif"
# pic3 = "room_3.gif"
# pic4 = "room_4.gif"
# scl_fctr = 7
# bot_size = (.5, .5)
# redraw = 4

# very Hard
# pic1 = "test1.gif"

# convert images to numeric grid
def scan(area):
    obs_loc = []
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
                obs_loc.append((i, j))
    # print(obs_loc)


    return grid, columns, rows, im, obs_loc

# have turtle draw room (this is a slow process for large rooms)
def draw_floor(grid):
    for m in range(columns):
        for n in range(rows):
            spy.setpos(m, n)
            if n == 0:
                spy.pendown()
            if grid[m, n] == 1:
                spy.color("black", "black")
            else:
                spy.color("white", "white")
            if n == rows - 1:
                spy.penup()

    x = 0
    y = 0
    spy.penup()
    spy.setpos(x, y)
    spy.pendown()
    perimeter = (2 * (rows - 1)) + (2 * (columns - 1))
    for i in range(perimeter - 1):
        if i >= 0 and i < columns - 1:
            x += 1
        elif i >= columns - 1 and i < (rows - 1 + columns - 1):
            y += 1
        elif i >= (rows - 1 + columns - 1) and i < (columns - 1 + rows - 1 + columns - 1):
            x -= 1
        else:
            y -= 1
        spy.setpos(x, y)
        if grid[x, y] == 1:
            spy.color("black")
        else:
            spy.color("white")

# determine which way the robot needs to go next.
def bot_point(spot, old_spot):
    face = (spot[0] - old_spot[0], spot[1] - old_spot[1])
    if face == (0, 1):
        pointer = 90
    elif face == (1, 0):
        pointer = 0
    elif face == (0, -1):
        pointer = 270
    else:
        pointer = 180
    return pointer

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

# classify neighbor nodes for D*
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
        ccd.setpos(bt_pos)
        path.append(bt_pos)
        if bt_pos in front:
            front.remove(bt_pos)
        if bt_pos == goal:
            prox.append(bt_pos)
            return
        bt_path.append(bt_pos)
        bt_explore(bt_pos, goal, bt_path, bt_next, grid, columns, rows)


# neighbor search for CCD*
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
    # if first[1] == end and len(front) > 1:
    #     backtrack(current_pos, grid, columns, rows, front, path, prox)
    #     return
    prox.append(first[1])


# explore neighbors for CCD*
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
    # out.show()
    out.save("out.png")


# run the CCD* planner
def ccd_star_plan(start, end, front, grid, columns, rows, im):
    prox = []  # a list of all the places left to explore
    path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
    ccd.clear()

    front.append(start)
    prox.append(start)

    # Get start time
    plan_runstart = time.time()
    ccd.penup()
    ccd.setpos(start)
    ccd.pendown()
    while len(front) != 0:

        current_pos = prox.pop()
        ccd.setpos(current_pos)
        if current_pos in front:
            front.remove(current_pos)
        path.append(current_pos)
        if current_pos == end:
            ccd.penup()
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
    front = []
    n = len(clean)
    for i in range(len(dirty) - len(clean)):
        front.append(dirty[n])
        n += 1
    return front

# identify new obstacle nodes
def new_obs(obs, obs1):

    for i in range(len(obs1)):
        if obs1[i] in obs:
            obs.remove(obs1[i])

# # draw new obstacles with turtle
def draw_nobs(obs, obs_old):
    spy.penup()
    if len(obs_old) > 1:
        spy.color("white")
        spy.setpos(obs_old[0])
        spy.pendown()
        for n in range(len(obs_old)):
            spy.setpos(obs_old[n])
        spy.penup()
    if len(obs) > 1:
        spy.color("black")
        spy.setpos(obs[0])
        spy.pendown()
        for m in range(len(obs)):
            spy.setpos(obs[m])
        spy.penup()



#### MAIN ####
# follow Path move turtle bot

start = (0, 1)
end = (0, 0)

front = []  # a list of where we haven't tried to go
clean = [] # a list of where we've been
# dirty = []

count = 1
output1 = mapcount(count)
grid1 = output1[0]
columns = output1[1]
rows = output1[2]
im1 = output1[3]
obs1 = output1[4]

wn = turtle.Screen()
wn.screensize(500, 500)
wn.setworldcoordinates(-1, rows + 1, columns + 1, -1)
wn.bgcolor("black")
wn.title("CCD")

bot = turtle.Turtle()
bot.shape('turtle')
bot.resizemode("user")
bot.shapesize(bot_size[0], bot_size[1])
bot.pensize(scl_fctr)
bot.speed(2)
bot.color("yellow", "black")

spy = turtle.Turtle()
spy.shape('circle')
spy.resizemode("auto")
spy.hideturtle()
spy.pensize(scl_fctr)
spy.speed(10)
spy.color("white")

ccd = turtle.Turtle()
ccd.shape('circle')
ccd.resizemode("auto")
ccd.hideturtle()
ccd.pensize(scl_fctr / 3)
ccd.speed(10)
ccd.color("blue")

draw_floor(grid1)

runstart = time.time()

dirty = ccd_star_plan(start, end, front, grid1, columns, rows, im1)
dirty1 = dirty

count = 2
output2 = mapcount(count)
grid = output2[0]
im = output2[3]
obs = output2[4]
new_obs(obs, obs1)
obs_old = []
draw_nobs(obs, obs_old)
obs_old = obs

spot = 0
i=0
bot.penup()
bot.setpos(dirty1[0])
bot.pendown()
old_spot = (-1, 1)
while spot != end:
    spot = dirty[i]
    pointer = bot_point(spot, old_spot)
    bot.setheading(pointer)
    bot.setpos(dirty[i])
    clean.append(spot)
    if spot == end:
         break
    #redraw map
    if i == redraw:
        output = mapcount(count)
        grid = output[0]
        im = output[3]
        obs = output[4]
        new_obs(obs, obs1)
        draw_nobs(obs, obs_old)
        obs_old = obs

    #look ahead
    ahead = dirty[i + 1]
    if grid[ahead[0], ahead[1]] == 1:
        # front = nob_front(clean, dirty1)
        dirty = ccd_star_plan(spot, end, front, grid, columns, rows, im)
        count = count + 1
        if count > 4:
            redraw = 20

        i = 0
    else:
        i = i + 1
    old_spot = spot

runstop = time.time()
print('Final Path Length ' + str(len(clean)))
print('TOTAL EXECUTION TIME FOR Planning: ' + str(runstop - runstart))

dupe = overlapped(clean)
visualize_search(im1, dirty1, dupe)
turtle.done()
