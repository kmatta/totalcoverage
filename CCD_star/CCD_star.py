from PIL import Image
import numpy
import queue as Q

white = (255, 255, 255)
black = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
PURPLE = (85, 26, 139)
YELLOW = (0, 50, 50)
GRAY = (100, 100, 100)

#area = "test.gif"
area = "Store.gif"

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

def explore(x, y, end, path, grid):

    # Check for boundary
    if x < 0 or x > rows - 1 or y < 0 or y > columns - 1:
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
        if not any((x, y) in item for item in dirty.queue):
            dirty.put((h, (x, y)))
        return
    else:
        print(grid[x, y])

def cost(x, y, end):
    h = (abs(x - end[0]) + abs(y - end[1])) * -1
    return h

def visualize_search():
    pixel_access = im.load()

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = GREEN
    pixel_access[end[0], end[1]] = RED

    # display and (maybe) save results
    im.show()

start = (0, 1)
end = (0, 0)

dirty = Q.PriorityQueue()  # priority q for planning algorithm
path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal

dirty.put((0, start))

while len(dirty.queue) != 0:

    current = dirty.get()
    current_pos = current[1]
    path.append(current_pos)
    if current_pos == end:
        break

    # look left
    explore(current_pos[0], current_pos[1] - 1, end, path, grid)
    # look up
    explore(current_pos[0] - 1, current_pos[1], end, path, grid)
    # look right
    explore(current_pos[0], current_pos[1] + 1, end, path, grid)
    # look down
    explore(current_pos[0] + 1, current_pos[1], end, path, grid)

visualize_search()
print(path)

