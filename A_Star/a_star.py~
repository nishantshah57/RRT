import sys
from PIL import Image
import copy

from Queue import PriorityQueue
import time

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file

'''
These variables determine display color, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)
RED = (255, 0, 0)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing nodes to expand to in the future


def heuristic(a, b):
	(x1, y1) = a
	(x2, y2) = b
	h = abs(x2 - x1) + abs(y2 - y1) # Manhattan Distance
	return h

def expand_nodes(map, size, node):
	x = node[0]
	y = node[1]
	next_nodes = []

	x_max = size[0]
	y_max = size[1]

	if (x+1 < x_max) and (map[x+1,y] != 0): # !=0 for circuit and !=1 for other .gif
		next_nodes.append((x+1,y))

	if (y+1 < y_max) and (map[x,y+1] != 0):
		next_nodes.append((x,y+1))

	if (y>=1) and (map[x,y-1] != 0):
		next_nodes.append((x,y-1))

	if (x>=1) and (map[x-1,y] != 0):
		next_nodes.append((x-1,y))

	return next_nodes

def A_star(map, size, start, goal):
	open = PriorityQueue(0)
	open.put((0, start))
	predecessor = {}
	explored = {}

	predecessor[start] = None
	explored[start] = 0
	G = 0
	while open.qsize():
		current_node = open.get()[1]

		if current_node == goal:
			G = open.get()[0]
			break

		successors = expand_nodes(map, size, current_node)
		for successor in successors:
			cost = explored[current_node] + 1
			# print("successor: {}".format(successor))
			if successor not in explored or cost < explored[successor] :
				explored[successor] = cost
				# print "Reached"
				total_cost = cost + heuristic(goal, successor)
				open.put((total_cost, successor))
				predecessor[successor] = current_node

	path = []
	while current_node != start:
		path.append(current_node)
		current_node = predecessor[current_node]
	path.append(start)
	path.reverse()

	frontier = {}
	for f in open.queue:
		frontier[f[1]] = f[0]

	return path, explored, frontier, G

def search(map, size):

	global path, start, end, path, expanded, frontier

	"""
	This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
	provided map.
	:param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
	"""
	print ("")
	print "Start Point: " + str(start)
	print "End   Point: " + str(end)

	# O is unoccupied (white); 1 is occupied (black)
	print ("")
	print "pixel value at start point ", map[start[0], start[1]]
	print "pixel value at end point ", map[end[0], end[1]]

	start_time = time.time()
	path, expanded, frontier, G = A_star(map, size, start, end)
	end_time = time.time()
	time_diff = end_time - start_time

	print("")
	print("Cost G:     ") + str(G) + ' units'
	print("Time Taken: ") + str(time_diff) + ' sec'

	visualize_search("out.png") # see what your search has wrought (and maybe save your next_nodes)

def visualize_search(save_file="do_not_save_img.png"):
	"""
	:param save_file: (optional) filename to save image to (no filename given means no save file)
	"""
	im = Image.open(difficulty).convert("RGB")
	pixel_access = im.load()

	# draw start and end pixels
	pixel_access[start[0], start[1]] = NEON_GREEN
	pixel_access[end[0], end[1]] = NEON_GREEN

	# draw expanded pixels
	for pixel in expanded.keys():
		pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

	# draw path pixels
	for pixel in path:
		pixel_access[pixel[0], pixel[1]] = NEON_GREEN

	 # draw frontier pixels
	for pixel in frontier.keys():
		pixel_access[pixel[0], pixel[1]] = RED

	# display and (maybe) save next_nodes
	im.show()
	if(save_file != "do_not_save.png"):
		im.save(save_file)
	# im.close()

if __name__ == "__main__":
	# Throw Errors && Such
	assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
	assert len(sys.argv) == 2, "Incorrect Number of arguments"      # require difficulty input

	# Parse input arguments
	function_name = str(sys.argv[0])
	difficulty = str(sys.argv[1])
	print ("")
	print ("A* Implementation")
	print "running " + function_name + " with " + difficulty + " difficulty."

	# Hard code start and end positions of search for each difficulty level
	if difficulty == "trivial.gif":
		start = (8, 1)
		end = (20, 1)
		im = Image.open(difficulty)
		search(im.load(), im.size)

	elif difficulty == "medium.gif":
		start = (8, 201)
		end = (110, 1)
		im = Image.open(difficulty)
		search(im.load(), im.size)

	elif difficulty == "hard.gif":
		start = (10, 1)
		end = (401, 220)
		im = Image.open(difficulty)
		search(im.load(), im.size)

	elif difficulty == "very_hard.gif":
		start = (1, 324)
		end = (580, 1)
		im = Image.open(difficulty)
		search(im.load(), im.size)

	elif difficulty == "circuit.jpg":
		start = (605, 741)
		end = (749, 13)
		im = Image.open(difficulty)
		im = im.convert('1')
		search(im.load(), im.size)
	else:
		assert False, "Incorrect difficulty level provided"
