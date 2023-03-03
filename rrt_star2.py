'''
Requirements:
pip install pygame
'''

import pygame as pg
import random
import sys, math, time
from anytree import Node


''' Window prefs '''
width = 1000				 # Window width
height = 1000				# Window height
pg.init()
surf = pg.display.set_mode((width, height), pg.RESIZABLE)
pg.display.set_caption("RRT* Test")
clock = pg.time.Clock()


''' Colours '''
BLACK = (0, 0, 0)
WHITE = (225, 225, 225)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BROWN = (150, 90, 62)
BLUE = (0, 0, 255)
GRAY = (100, 100, 100)


''' Settings '''
limitFPS = True			# Limits FPS
limit = 60				 # FPS limit

''' Drone settings '''
jitter = 3						# resolution for drone positioning
easing = 0.18					# decelerating amount
max_speed = 6
acceleration = 0.5
distance = 50					# decelerating distance
drone_size = 30					# size of the drone collision

''' RRT settings '''
nodes = []
path = []
node_max_distance = 600			# area for RRT* to consider nodes
node_min_distance = 20			# max distance between nodes
node_ignore_distance = 3
path_iterations = 800			# number of nodes


############################################################################################################


class Drone():
	
	def __init__(self, x, y, xv, yv, acc=acceleration, easing=easing, max_speed=max_speed, size=drone_size):
		self.x = x
		self.y = y
		self.xv = xv
		self.yv = yv
		self.acc = acc
		self.max_speed = max_speed
		self.easing = easing
		self.size = size

	def move(self, coords):
		delta_x = coords[0] - self.x
		delta_y = coords[1] - self.y
		delta_r = math.sqrt(delta_x**2 + delta_y**2)

		# if distance to travel too small
		if delta_r < jitter:
			self.xv = 0
			self.yv = 0
			return
		
		# getting the angle
		theta = math.atan2(delta_y, delta_x)
		
		# adding acceleration
		if math.sqrt(self.xv**2 + self.yv**2) < self.max_speed:
			self.xv += self.acc * math.cos(theta)
			self.yv += self.acc * math.sin(theta)

		# decelerating
		if delta_r < distance or not (math.atan2(self.yv, self.xv) < theta+0.1 and math.atan2(self.yv, self.xv) > theta-0.1):
			self.xv *= 1 - self.easing
			self.yv *= 1 - self.easing

		# moving the drone
		self.x += self.xv
		self.y += self.yv


class Node():
	
	def __init__(self, x, y, parent, cost):
		self.x = x
		self.y = y
		self.parent = parent
		self.cost = cost
	
	def path(self):
		l = []
		node = self
		while True:
			l.append(node)
			if node.parent:
				node = node.parent
			else:
				l.reverse()
				return l

############################################################################################################


# creates drone
drone = Drone(100, 100, 0, 0)
nodes.append(Node(drone.x, drone.y, None, 0))


# creates obsticles randomly
obsticles = []
for i in range(10):
	x = random.randint(200, width-50)
	y = random.randint(50, height-50)
	size_x = random.randint(50, 100)
	size_y = random.randint(50, 300)
	obsticles.append(pg.Rect(x, y, size_x, size_y))


############################################################################################################


''' Creates a node in random direction '''
def create_node():
	random_x = random.randint(0, width)
	random_y = random.randint(0, height)

	closest_node = nodes[0]

	# searches for the closest node
	for node in nodes:
		if node != nodes[1]:
			node_distance = math.sqrt((node.x - random_x)**2 + (node.y - random_y)**2)
			if math.sqrt((closest_node.x - random_x)**2 + (closest_node.y - random_y)**2) > node_distance:
				closest_node = node
	
	x = 0
	y = 0
	delta_x = random_x - closest_node.x
	delta_y = random_y - closest_node.y
	delta_r = math.sqrt(delta_x**2 + delta_y**2)
	theta = math.atan2(delta_y, delta_x)
	test_x = closest_node.x
	test_y = closest_node.y

	# Check collisions
	for _ in range(int(node_min_distance/10)):
		test_x += 10 * math.cos(theta)
		test_y += 10 * math.sin(theta)
		
		for obs in obsticles:
			if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
												or obs.collidepoint(test_x, test_y+drone.size)
												or obs.collidepoint(test_x+drone.size, test_y+drone.size)
												or obs.collidepoint(test_x-drone.size, test_y)
												or obs.collidepoint(test_x, test_y-drone.size)
												or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
				# x = test_x
				# y = test_y
				return

	# makes the distance not longer than the min distance
	if delta_r > node_min_distance:
		x = (node_min_distance * math.cos(theta)) + closest_node.x
		y = (node_min_distance * math.sin(theta)) + closest_node.y
	elif delta_r < node_ignore_distance:
		return None
	else:
		x = random_x
		y = random_y

	cost = math.sqrt((x - closest_node.x)**2 + (y - closest_node.y)**2)

	nodes.append(Node(x, y, closest_node, closest_node.cost + cost))
	return nodes[-1]


''' Creates a node in random direction '''
def create_node2():
	random_x = random.randint(0, width)
	random_y = random.randint(0, height)

	closest_node = nodes[0]

	# searches for the closest node
	for node in nodes:
		if node != nodes[1]:
			node_distance = math.sqrt((node.x - random_x)**2 + (node.y - random_y)**2)
			if math.sqrt((closest_node.x - random_x)**2 + (closest_node.y - random_y)**2) > node_distance:
				closest_node = node
	
	x = 0
	y = 0
	delta_x = random_x - closest_node.x
	delta_y = random_y - closest_node.y
	delta_r = math.sqrt(delta_x**2 + delta_y**2)
	theta = math.atan2(delta_y, delta_x)
	test_x = closest_node.x
	test_y = closest_node.y

	# Check collisions
	for _ in range(int(node_min_distance/10)):
		test_x += 10 * math.cos(theta)
		test_y += 10 * math.sin(theta)
		
		for obs in obsticles:
			if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
												or obs.collidepoint(test_x, test_y+drone.size)
												or obs.collidepoint(test_x+drone.size, test_y+drone.size)
												or obs.collidepoint(test_x-drone.size, test_y)
												or obs.collidepoint(test_x, test_y-drone.size)
												or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
				# x = test_x
				# y = test_y
				return

	# makes the distance not longer than the min distance
	if delta_r > node_min_distance:
		x = (node_min_distance * math.cos(theta)) + closest_node.x
		y = (node_min_distance * math.sin(theta)) + closest_node.y
	elif delta_r < node_ignore_distance:
		return None
	else:
		x = random_x
		y = random_y

	cost = math.sqrt((x - closest_node.x)**2 + (y - closest_node.y)**2)

	node = Node(x, y, closest_node, closest_node.cost + cost)

	parent_node = update_node(node)

	if parent_node:
		node.parent = parent_node
		cost = math.sqrt((x - parent_node.x)**2 + (y - parent_node.y)**2)
		node.cost = parent_node.cost + cost

	nodes.append(node)
	return nodes[-1]


''' Finds a more cost effective parent '''
def update_nodes():
	for node in nodes[1::]:
		for comparing_node in nodes:
			if comparing_node == node:
				continue
			elif comparing_node.parent == node:
				continue
			elif comparing_node.parent == nodes[0]:
				continue

			delta_distance = math.sqrt((comparing_node.x - node.x)**2 + (comparing_node.y - node.y)**2)

			if delta_distance < node_max_distance and node.cost > comparing_node.cost + delta_distance:
				delta_x = comparing_node.x - node.x
				delta_y = comparing_node.y - node.y
				theta = math.atan2(delta_y, delta_x)
				dist = math.sqrt(delta_x**2 + delta_y**2)
				ignore = 0
				test_x = node.x
				test_y = node.y

				for i in range(int(dist/10)):
					test_x += 10 * math.cos(theta)
					test_y += 10 * math.sin(theta)
					
					for obs in obsticles:
						if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
															or obs.collidepoint(test_x, test_y+drone.size)
															or obs.collidepoint(test_x+drone.size, test_y+drone.size)
															or obs.collidepoint(test_x-drone.size, test_y)
															or obs.collidepoint(test_x, test_y-drone.size)
															or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
							ignore = 1
							break
					
					if ignore:
						break

				if not ignore:
					node.parent = comparing_node


''' Updates parent of one node in path '''
def update_node_in_path(node):
	for comparing_node in path:
		if comparing_node == node:
			continue
		elif comparing_node.parent == node:
			continue
		elif comparing_node.parent == nodes[0]:
			continue

		delta_distance = math.sqrt((comparing_node.x - node.x)**2 + (comparing_node.y - node.y)**2)

		if delta_distance < node_max_distance and node.cost > comparing_node.cost + delta_distance:
			delta_x = comparing_node.x - node.x
			delta_y = comparing_node.y - node.y
			theta = math.atan2(delta_y, delta_x)
			dist = math.sqrt(delta_x**2 + delta_y**2)
			ignore = 0
			test_x = node.x
			test_y = node.y

			for i in range(int(dist/5)):
				test_x += 5 * math.cos(theta)
				test_y += 5 * math.sin(theta)
				
				for obs in obsticles:
					if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
														or obs.collidepoint(test_x, test_y+drone.size)
														or obs.collidepoint(test_x+drone.size, test_y+drone.size)
														or obs.collidepoint(test_x-drone.size, test_y)
														or obs.collidepoint(test_x, test_y-drone.size)
														or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
						ignore = 1
						break
					
				if ignore:
					break

			if not ignore:
				return comparing_node
	
	return None


''' Updates parent of one node in path '''
def update_node(node):
	for comparing_node in nodes:
		if comparing_node == node:
			continue
		elif comparing_node == nodes[1]:
			continue
		elif comparing_node.parent == node:
			continue
		elif comparing_node.parent == nodes[0]:
			continue

		delta_distance = math.sqrt((comparing_node.x - node.x)**2 + (comparing_node.y - node.y)**2)

		if delta_distance < 200 and node.cost > comparing_node.cost + delta_distance:
			delta_x = comparing_node.x - node.x
			delta_y = comparing_node.y - node.y
			theta = math.atan2(delta_y, delta_x)
			dist = math.sqrt(delta_x**2 + delta_y**2)
			ignore = 0
			test_x = node.x
			test_y = node.y

			for i in range(int(dist/5)):
				test_x += 5 * math.cos(theta)
				test_y += 5 * math.sin(theta)
				
				for obs in obsticles:
					if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
														or obs.collidepoint(test_x, test_y+drone.size)
														or obs.collidepoint(test_x+drone.size, test_y+drone.size)
														or obs.collidepoint(test_x-drone.size, test_y)
														or obs.collidepoint(test_x, test_y-drone.size)
														or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
						ignore = 1
						break
					
				if ignore:
					break

			if not ignore:
				return comparing_node
	
	return None


''' Updates just the path '''
def update_path():
	local_path = []
	if path:
		while True:
			node = update_node_in_path(path[-1])
			if node:
				path[-1].parent = node
				local_path.append(path[-1])
			else:
				local_path.append(path[-1])
			
			for i in range(len(path)-1, 0, -1):
				if path[i] != local_path[-1].parent:
					path.pop(i)
				else:
					break

			if len(path) == 1:
				local_path.reverse()
				return local_path


''' Finds path to the target '''
def find_path():
	while True:
		node = create_node()

		if node == None:
			continue
	
		if math.sqrt((nodes[1].x - node.x)**2 + (nodes[1].y - node.y)**2) < node_max_distance:
			delta_x = nodes[1].x - node.x
			delta_y = nodes[1].y - node.y
			theta = math.atan2(delta_y, delta_x)
			dist = math.sqrt(delta_x**2 + delta_y**2)
			ignore = 0
			test_x = node.x
			test_y = node.y
			for i in range(int(dist/5)):
				test_x += 5 * math.cos(theta)
				test_y += 5 * math.sin(theta)
				
				for obs in obsticles:
					if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
														or obs.collidepoint(test_x, test_y+drone.size)
														or obs.collidepoint(test_x+drone.size, test_y+drone.size)
														or obs.collidepoint(test_x-drone.size, test_y)
														or obs.collidepoint(test_x, test_y-drone.size)
														or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
						ignore = 1
						break
					
				if ignore:
					break

			if not ignore:
				nodes[1].parent = node
				nodes[1].cost = node.cost + dist
				return
		
		# update_nodes()
		show_nodes()
		pg.display.flip()


''' Finds path to the target '''
def find_path2():
	tries = 0
	found = 0
	global nodes, path
	while True:
		node = create_node2()

		if node == None:
			continue
	
		if math.sqrt((nodes[1].x - node.x)**2 + (nodes[1].y - node.y)**2) < node_max_distance:
			delta_x = nodes[1].x - node.x
			delta_y = nodes[1].y - node.y
			theta = math.atan2(delta_y, delta_x)
			dist = math.sqrt(delta_x**2 + delta_y**2)
			ignore = 0
			test_x = node.x
			test_y = node.y
			for i in range(int(dist/10)):
				test_x += 10 * math.cos(theta)
				test_y += 10 * math.sin(theta)
				
				for obs in obsticles:
					if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
														or obs.collidepoint(test_x, test_y+drone.size)
														or obs.collidepoint(test_x+drone.size, test_y+drone.size)
														or obs.collidepoint(test_x-drone.size, test_y)
														or obs.collidepoint(test_x, test_y-drone.size)
														or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
						ignore = 1
						break
					
				if ignore:
					break

			if not ignore:
				found = 1
				if nodes[1].cost > node.cost + dist or nodes[1].cost == -1:
					nodes[1].parent = node
					nodes[1].cost = node.cost + dist
		
		if tries >= path_iterations and found:
			return
		elif tries >= path_iterations and not found:
			path = []
			nodes = []
			nodes.append(Node(drone.x, drone.y, None, 0))
			print("No path found")
			return
		
		tries += 1
		# update_nodes()
		show_nodes()
		pg.display.flip()


''' Draws the nodes '''
def show_nodes():
	for node in nodes:
		if node.parent != None:
			pg.draw.line(surf, BLACK, (node.x, node.y), (node.parent.x, node.parent.y), width=3)


''' Draws the path '''
def show_path():
	for node in path:
		if node.parent != None:
			pg.draw.line(surf, GREEN, (node.x, node.y), (node.parent.x, node.parent.y), width=4)


''' Moves the drone following the path '''
def fly_path():
	global path, nodes
	if math.sqrt((path[0].x - drone.x)**2 + (path[0].y - drone.y)**2) < jitter:
		path.pop(0)
	
	if len(path) == 0:
		path = []
		nodes = []
		nodes.append(Node(drone.x, drone.y, None, 0))
		return

	drone.move((path[0].x, path[0].y))


############################################################################################################


target_node = None
while True:
	if limitFPS:
		''' Limit FPS '''
		clock.tick(limit)
		# fps = clock.get_fps()
		# print("	   " + str(int(fps)) + "	   ", end='\r')
	else:
		clock.tick()
		# fps = clock.get_fps()
		# print("	   " + str(int(fps)) + "	   ", end='\r')

	for event in pg.event.get():
		''' Exit on X '''
		if event.type == pg.QUIT:
			pg.quit()
			sys.exit()

		if event.type == pg.MOUSEBUTTONDOWN:
			''' Left click '''
			if event.button == 1:
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = Node(coords[0], coords[1], None, -1)
				nodes.append(target_node)
				find_path()
				# update_nodes()
				path = target_node.path()
				path = update_path()
				print("--- %s seconds ---" % (time.time() - start_time))

			''' Middle click '''
			if event.button == 2:
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = Node(coords[0], coords[1], None, -1)
				nodes.append(target_node)
				find_path2()
				# update_nodes()
				update_nodes()
				path = target_node.path()
				if len(path) == 1:
					path = []
					nodes = []
					nodes.append(Node(drone.x, drone.y, None, 0))
				# path = update_path()
				print("--- %s seconds ---" % (time.time() - start_time))

			''' Right click '''
			if event.button == 3:
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = Node(coords[0], coords[1], None, -1)
				nodes.append(target_node)
				find_path2()
				path = target_node.path()
				if len(path) == 1:
					path = []
					nodes = []
					nodes.append(Node(drone.x, drone.y, None, 0))
				path = update_path()
				print("--- %s seconds ---" % (time.time() - start_time))

	surf.fill(GRAY)

	for obs in obsticles:
		pg.draw.rect(surf, BLACK, obs)

	show_nodes()
	
	if path:
		pg.draw.circle(surf, YELLOW, [target_node.x, target_node.y], 10)
		show_path()
		fly_path()

	pg.draw.circle(surf, RED, [drone.x, drone.y], drone.size)
	pg.draw.circle(surf, BLUE, [drone.x, drone.y], 10)
	pg.display.flip()