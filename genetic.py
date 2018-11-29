import sys
from PIL import Image
import copy
import math
import time
import random

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)  # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = ""  # a string reference to the original import file

'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}  # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}  # a dictionary of (x,y) tuples, representing nodes to expand to in the future
A = 1 #global constants
B = 100
C = 1

def search(map, size):
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """

    # 1 is unoccupied (white); 0 is occupied (black)
    print ("pixel value at start point ", map[start[0], start[1]])
    print ("pixel value at end point ", map[end[0], end[1]])

    # put your final path into this array (so visualize_search can draw it in purple)
    # path.extend([(8,2), (8,3), (8,4), (8,5), (8,6), (8,7)])

    # put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
    # expanded.update({(7,2):True, (7,3):True, (7,4):True, (7,5):True, (7,6):True, (7,7):True})

    # put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
    # frontier.update({(6,2):True, (6,3):True, (6,4):True, (6,5):True, (6,6):True, (6,7):True})


    start1 = time.clock()
    # Begin
    Ps = start #start position
    Pe = end #end position

    minipath = []  # mini-path consisting of (x,y) pairs of length=path_size
    path_size = 5  # size of mini-path as determined by sensor range
    nbr_paths = 28
    count = 0

    #Dist = []  #total distance of mini-path by Eucledean
    #Free = []  #number of consecutive unclean cells in mini-path
    clean = []  #global path of cleaned (x,y) pairs
    node = Ps #current position of robot

    # Initialization
    # Create 8 random minipaths from start point to seed population
    minipath = get_minipaths(map, size, node, path_size, 8)

    # Genetic Motion Planner
    while (count != 1000) and (Pe not in clean):
        count += 1
        print("# OF GENERATIONS>>", count)

        # Selection
        # Select 2 minipaths for further exploration
        # Get fitness score of all minipaths
        fitpath = []
        for i in range(0, len(minipath)):
            fpath = minipath[i]
            fitpath.append(fitness(fpath, clean))
        #print("fitpath:", fitpath)

        # Get two fittest paths
        f1 = fitpath.index(max(fitpath))
        parent1 = minipath[f1]
        del minipath[f1]
        del fitpath[f1]
        f2 = fitpath.index(max(fitpath))
        parent2 = minipath[f2]
        del minipath[f2]
        del fitpath[f2]
        #print("parent1", parent1)
        #print("parent2", parent2)

        # 50% chance of crossover or mutation
        # if random.randint(0, 1) == 1:
        #     # Crossover
        #     child1, child2 = crossover(parent1, parent2)
        # else:
        #     child1 = parent1
        #     child2 = parent2
        #
        # print("child1", child1)
        # print("child2", child2)
        #
        # if random.randint(0, 1) == 1:
        #     # Mutation
        #     child1 = mutation(child1, map, size)
        #     child2 = mutation(child2, map, size)


        # Add fittest path to global route
        if not (parent1 in clean):
            clean.extend(parent1)

        print('Clean=', clean)
        node = clean[-1]  # update current position of robot
        print('Node=', node)
        minipath = []
        minipath = get_minipaths(map, size, node, path_size, nbr_paths)

        minipath.append(parent1)
        minipath.append(parent2)
        print("minipath", minipath)

        path.extend(clean)
        #print(path)

    print ("Number of Nodes visited by Genetic: ", len(path))
    print ("Time in sec taken for Genetic: ", "{0:.2f}".format(time.clock() - start1))
    visualize_search("out.png")  # see what your search has wrought (and maybe save your results)


# function that returns all unique minipaths originating from a node
def get_minipaths(map,size,node,path_size,nbr_paths):
    minipaths = []  # array of all minipaths
    nbrpaths = nbr_paths # max number of efficient paths
    while len(minipaths) < nbrpaths:
        mpath = []  # array of current minipath
        mpath.append(node)
        while len(mpath) < path_size:  # append neighbors to node until equal to minipath size
            next = successors(map, size, mpath[-1])  # get neighbors of node
            print("successors:", next)
            print(mpath)
            #delete successors that are already in mpath

            result = next in mpath #all(elem in next for elem in mpath)  # true if all neighbors already in minipath
            print(result)
            if next == [] or result:  # new minipaths not possible
                nbrpaths = 0
                break
            gene = random.choice(next)  # select random neighbor
            print(gene)
            if gene not in mpath:  # add to minipath if not already in it
                mpath.append(gene)
                print("mpath:", mpath)
        if (len(mpath) == path_size) and not (mpath in minipaths):  # validate minipath size and uniqueness and add to list
            minipaths.append(mpath)
            print("minipaths:",minipaths)
        else:
            nbrpaths -= 1
            print(nbrpaths)

    return minipaths


def fitness(ipath,cleaned):
    #ipath is a minipath of length=path_size
    #cleaned is an array of visited nodes
    Dist = 0
    Free = 0
    Distc = 0
    #print("ipath", ipath)
    for j in range(len(ipath)-1):
        #print(j)
        #print(ipath[j])
        Dist = Dist + 1 #eucledean(ipath[j],ipath[j+1])
        if not (ipath[j+1] in cleaned):
            Free = Free + 1
            Distc += abs(ipath[0][0] - ipath[j+1][0])

    F = A*Dist + B*Free + C*Distc + eucledean(start, ipath[0])
    return F


def crossover(parent1,parent2):
    #parents are mini-paths of length=path_size
    #single crossover point chosen where valid
    child1 = []
    child2 = []
    for i in range(len(parent1)):
        if (i != 0) and (parent1[i] == parent2[i]):
            #found crossover point
            child1 = parent1[0:i]
            child1.extend(parent2[i:])

            child2 = parent2[0:i]
            child2.extend(parent1[i:])

            return child1, child2

    return parent1, parent2


def mutation(child,map,size):
    #single gene is selected at random
    point = random.randint(0, len(child)-1)
    #print("child", child)
    #find neighbors of random gene
    neighbors = successors(map,size,child[point])
    precedents = []
    #sort out which neighbors are not already in child
    for i in neighbors:
        if i not in child:
            if point == 0: #edge case
                if eucledean(i,child[point+1]) == math.sqrt(2):
                    precedents.append(i)
            elif point == (len(child)-1): #edge case
                if eucledean(i,child[point-1]) == math.sqrt(2):
                    precedents.append(i)
            else:
                precedents.append(i)

    #random precedent selected

    if precedents:
        xpoint = random.randint(0, len(precedents)-1)
        child[point] = precedents[xpoint] #mutate random gene with new gene
        #print ("mutantchild", child)
    return (child)


def eucledean(a, b):
    # Eucledean distance
    #print("a=", a, "b=", b)
    dist = abs(math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))
    if dist > 0:
        return dist
    else:
        return 0


# function to find neighbours of a given point (child nodes)
def successors(map, size, v):
    #print("v=", v)
    x = v[0]
    y = v[1]
    grid = size[0];
    points = []
    a = b = c = d = True

    # edge conditions
    if x < 1:
        a = False
    if x > grid - 2:
        b = False
    if y < 1:
        c = False
    if y > grid - 2:
        d = False

    # this is how priorities are set (order matters here)
    if b and (map[x + 1, y] == 1 or map[x + 1, y] == (255,255,255)):
        points.append((x + 1, y))

    if d and (map[x, y + 1] == 1 or map[x, y + 1] == (255,255,255)):
        points.append((x, y + 1))

    if c and (map[x, y - 1] == 1 or map[x, y - 1] == (255,255,255)):
        points.append((x, y - 1))

    if a and (map[x - 1, y] == 1 or map[x - 1, y] == (255,255,255)):
        points.append((x - 1, y))

    ## uncomment this to remove priority
    # shuffle(points)

    return points


def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # display and (maybe) save results
    im.show()
    if (save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    assert sys.version_info[0] == 2  # require python 2 (instead of python 3)
    assert len(sys.argv) == 2, "Incorrect Number of arguments"  # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print "running " + function_name + " with " + difficulty + " difficulty."

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "custom.gif":
        start = (6, 50)
        end = (155, 6)
    elif difficulty == "Test_grid.jpg":
        start = (1, 1)
        end = (99, 99)
    elif difficulty == "test1.gif":
        start = (0, 0)
        end = (400, 560)
    elif difficulty == "test2.gif":
        start = (0, 0)
        end = (230, 230)
    elif difficulty == "test3.gif":
        start = (0, 0)
        end = (130, 130)
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    sized = im.size
    search(im.load(), sized)

