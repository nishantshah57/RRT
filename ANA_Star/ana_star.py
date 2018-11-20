import sys
from PIL import Image
import copy
import time
from Queue import PriorityQueue

class ReversePriorityQueue(PriorityQueue):

	def put(self, tup):
		newtup = tup[0] * -1, tup[1], tup[2]
		PriorityQueue.put(self, newtup)

	def get(self):
		tup = PriorityQueue.get(self)
		newtup = tup[0] * -1, tup[1], tup[2]
		return newtup

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file

'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)
RED = (255,0,0)

G = 1e15
E = 1e15
'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing nodes to expand to in the future

# action = [[-1, 0 ], # go up
#          [ 0, -1], # go left
#          [ 1, 0 ], # go down
#          [ 0, 1 ]] # go right

def heuristic(a,b):
	(x1,y1) = a
	(x2,y2) = b
	h = abs(x2 - x1) + abs(y2 - y1) # Manhattan Distance
	return h

def compute_e(G, g, h):
	e = (G - g)/(h + 1e-15)
	return e

def expand_nodes(map, size, node):
	global action
	x = node[0]
	y = node[1]
	next_nodes = []
	x_max = size[0]
	y_max = size[1]

	if (x+1 < x_max) and (map[x+1,y] != 0):   # '!=0' for 'map_1' and 'crazy' &  '!=1' for 'medium, hard & very hard gifs'
		next_nodes.append((x+1,y))

	if (y+1 < y_max) and (map[x,y+1] != 0):
		next_nodes.append((x,y+1))

	if (y>=1) and (map[x,y-1] != 0):
		next_nodes.append((x,y-1))

	if (x>=1) and (map[x-1,y] != 0):
		next_nodes.append((x-1,y))
	# for i in range(len(action)):
	# 	x1 = x + action[i][0]
	# 	y1 = y + action[i][1]
	# 	if x1 >= 0 and x1 < x_max and y1 >=0 and y1 < y_max:
	# 		if (map[x1,y1] != 1): # Check occupied or not
	# 			next_nodes.append((x1,y1))

	return next_nodes

def prune(open, G, goal):
	update_frontier = ReversePriorityQueue(0)

	while not (open.empty()):
		node = open.get()
		e_s = node[0]
		g_s = node[1]
		state = node[2]

		h_s = heuristic(state, goal)

		if (g_s + h_s) < G:
			new_e_s = compute_e(G, g_s, h_s)
			update_frontier.put((new_e_s, g_s, state))

	return update_frontier

def print_stats(time_taken, G, E, sub_opt_cnt):
	print ""
	print "Sub-optimal Count: " + str(sub_opt_cnt)
	print "Cost G: " + str(G) + ' units'
	print "Sub-optimality: " + str(E) + ' units'
	print "Time Taken: " + str(time_taken) + ' sec'

	return time_taken

def improve_solution(predecessor, explored, open, G, E, map, size, start, goal):
	while not open.empty():
		current_node = open.get()
		# print(current_node)
		e_s = current_node[0]
		g_s = current_node[1]
		state =  current_node[2]

		if e_s < E:
			E = e_s

		if state == goal: # Checking goal is occupied or not
			G = g_s
			break

		successors = expand_nodes(map, size, state)
		for successor in successors:
			cost = explored[state] + 1
			# print("successor: {}".format(successor))
			if (successor not in explored or cost < explored[successor]):# or :
				explored[successor] = cost
				h_successor = heuristic(successor, goal)
				total_cost = cost + h_successor

				if (total_cost < G):
					e_successor = compute_e(G, cost, h_successor)
					open.put((e_successor, cost, successor))
				predecessor[successor] = state

	return predecessor, explored, open, G, E

def ANA_star(map, size, start, goal):
	global G, E
	sub_opt_cnt = 0
	g_start = 0
	h_start = heuristic(start, goal)
	e_start = compute_e(G, g_start, h_start)

	open = ReversePriorityQueue(0)
	open.put((e_start, g_start, start))
	predecessor = {}
	explored = {}

	predecessor[start] = None
	explored[start] = 0

	prev_time = 0

	while not open.empty():
		sub_opt_cnt += 1
		start_time = time.time()
		predcessor, explored, open, G, E = improve_solution(predecessor, explored, open, G, E, map, size, start, goal)
		end_time = time.time()
		time_diff = end_time - start_time

		prev_time = print_stats(time_diff + prev_time, G, E, sub_opt_cnt)
		open = prune(open, G, goal)

	path = []
	current_node = goal
	while (current_node != start):
		path.append(current_node)
		current_node = predecessor[current_node]
	path.append(start)
	path.reverse()

	frontier = {}
	# print("OPEN: {}".format(open.queue))
	for f in open.queue:
		frontier[f[2]] = f[1]

	return path, explored, frontier, sub_opt_cnt

def search(map, size):
	global path, start, end, expanded, frontier
	"""
	This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
	provided map.
	:param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
	"""
	# O is unoccupied (white); 1 is occupied (black)
	print "pixel value at start point ", map[start[0], start[1]]
	print "pixel value at end point ", map[end[0], end[1]]
	print "Reached"

	# # put your final path into this array (so visualize_search can draw it in purple)
	# path.extend([(8,2), (8,3), (8,4), (8,5), (8,6), (8,7)])
	#
	# # put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
	# expanded.update({(7,2):True, (7,3):True, (7,4):True, (7,5):True, (7,6):True, (7,7):True})
	#
	# # put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
	# frontier.update({(6,2):True, (6,3):True, (6,4):True, (6,5):True, (6,6):True, (6,7):True})

	#------------------------------------------------------------------------------------------
	# ANA Star Search
	path, expanded, frontier, sub_opt_cnt = ANA_star(map, size, start, end)
	#-------------------------------------------------------------------------------------------


	visualize_search("out.png") # see what your search has wrought (and maybe save your results)


def visualize_search(save_file="do_not_save.png"):
	"""
	:param save_file: (optional) filename to save image to (no filename given means no save file)
	"""
	# fp = open(difficulty,'rb')
	# im = Image.open(fp).convert("RGB")
	im = Image.open(difficulty).convert("RGB")
	# with Image.open(difficulty).convert("RGB") as im:
	pixel_access = im.load()

	# draw start and end pixels
	pixel_access[start[0], start[1]] = NEON_GREEN
	pixel_access[end[0], end[1]] = NEON_GREEN

	# draw path pixels
	# print(explored)

	# draw frontier pixels
	for pixel in frontier.keys():
		pixel_access[pixel[0], pixel[1]] = RED

	# draw expanded pixels
	for pixel in expanded.keys():
		pixel_access[pixel[0], pixel[1]] = DARK_GRAY

	for pixel in path:
		# print(pixel_access[pixel[0], pixel[1]])
		pixel_access[pixel[0], pixel[1]] = NEON_GREEN


	# display and (maybe) save results
	im.show()
	print "Reached"
	if(save_file != "do_not_save.png"):
		im.save(save_file)
	# im.close()
	# fp.close()

if __name__ == "__main__":
	# Throw Errors && Such
	# global difficulty, start, end
	assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
	assert len(sys.argv) == 2, "Incorrect Number of arguments"      # require difficulty input

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
	elif difficulty == "circuit.jpg":
		start = (605, 741)
		end = (749, 13)
	else:
		assert False, "Incorrect difficulty level provided"

	# Perform search on given image
	im = Image.open(difficulty)
	im = im.convert('1')
	search(im.load(), im.size)
