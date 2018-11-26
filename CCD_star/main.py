import sys
from PIL import Image
import math
import queue as Q
import time
# import matplotlib.pyplot as plt
import copy

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)  # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = ""  # a string reference to the original import file

'''
These variables determine display color, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
RED = (255, 0, 0)
PURPLE = (85, 26, 139)
YELLOW = (0, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}  # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}  # a dictionary of (x,y) tuples, representing nodes to expand to in the future
dirty = Q.PriorityQueue()  # priority q for planning algorithim



def search(map):
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """

    # O is unoccupied (white); 1 is occupied (black)
    print("pixel value at start point ", map[start[0], start[1]])
    print("pixel value at end point ", map[end[0], end[1]])

    dirty.put((0, start))

    while len(dirty.queue) != 0:
        current = dirty.get()
        current_pos = current[1]
        if current_pos == end:
            break
        path.append(current_pos)
        # look left
        explore(current_pos[0], current_pos[1] - 1, end, path, map)
        # look up
        explore(current_pos[0] - 1, current_pos[1], end, path, map)
        # look right
        explore(current_pos[0], current_pos[1] + 1, end, path, map)
        # look down
        explore(current_pos[0] + 1, current_pos[1], end, path, map)

    print(path)

    visualize_search("out.png")  # see what your search has wrought (and maybe save your results)

#def revisit():

def cost(x, y, end):
    h = (abs(x - end[0]) + abs(y - end[1])) * -1
    return h

def explore(x, y, end, path, map):
    # Check for boundary
    if x < 0 or x > rows - 1 or y < 0 or y > columns - 1:
        return
    # Check if the node has already been explored
    elif (x, y) in path:
        return
    # check for obstacle
    elif map[x, y] == 1:
        return
    # Check if path clear
    elif map[x, y] == 0:
        h = cost(x, y, end)
        dirty.put((h, (x, y)))
        return
    else:
        print("Dead end")




def visualize_search(save_file="solved_trivial.gif"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()


    # draw frontier pixels
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = YELLOW

    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = RED

    # display and (maybe) save results
    im.show()
    if (save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    assert sys.version_info[0] == 3  # require python 3 (instead of python 2)
    assert len(sys.argv) == 2, "Incorrect Number of arguments"  # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print("running " + function_name + " with " + difficulty + " difficulty.")

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 0)
        end = (20, 0)
        rows = 22
        columns = 22
    elif difficulty == "test.gif":
        start = (0, 1)
        end = (0, 0)
        rows = 10
        columns = 10
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    search(im.load())
