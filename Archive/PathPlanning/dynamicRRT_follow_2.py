'''
Requirements:
pip install pygame
'''

import pygame as pg
import random
import sys, math, time
from djitellopy import Tello

tello = Tello()

''' Window prefs '''
width = 1000				 # Window width
height = 900				# Window height
pg.init()
surf = pg.display.set_mode((width, height), pg.RESIZABLE)
pg.display.set_caption("Dynamic RRT* Test")
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
limit = 30				 # FPS limit

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
node_max_distance = 40			# max distance between nodes
node_min_distance = 4			# min distance between nodes
node_neighbour_distance = 150
path_iterations = 800			# number of nodes
obsticle_detection_resolution = 10
obsticle_avoidance_size = 5


############################################################################################################


class Drone():
	
	def __init__(self, x, y, xv, yv, acc=acceleration, easing=easing, max_speed=max_speed, size=drone_size, dir=0):
		self.x = x
		self.y = y
		self.xv = xv
		self.yv = yv
		self.acc = acc
		self.max_speed = max_speed
		self.easing = easing
		self.size = size
		self.dir = dir

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
	
	def __init__(self, x, y, parent=None, cost=None):
		self.x = x
		self.y = y
		self.parent = parent
		self.cost = cost
		self.children = set()
	
	def path(self):
		l = []
		node = self
		for i in range(100):
			l.append(node)
			if node.parent:
				node = node.parent
			elif node.parent == None:
				l.reverse()
				return l
		return []
	
	def add_parent(self, parent):
		if self.parent:
			self.parent.children.discard(self)
		self.parent = parent
		parent.children.add(self)
		distance = math.sqrt((self.x-parent.x)**2 + (self.y-parent.y)**2)
		self.cost = parent.cost + distance


class DynamicRRT():

	def __init__(self, start, goal,
				 area, maxIter=200):

		self.start = Node(start[0], start[1])
		self.end = Node(goal[0], goal[1])
		self.Xarea = area[0]
		self.Yarea = area[1]
		self.maxIter = maxIter

	def get_random_point(self):
		return [random.randint(0, self.Xarea), random.randint(0, self.Yarea)]
		
	def check_point_collision(self, node, obsticle_list):
		for obs in obsticle_list:
			# pygame collision
			# if obs.collidepoint(node.x, node.y):
			# 	return False
			
			# vanilla collision
			x, y, x_size, y_size = obs.left-obsticle_avoidance_size, obs.top-obsticle_avoidance_size, obs.width+(obsticle_avoidance_size*2), obs.height+(obsticle_avoidance_size*2)
			if node.x > x and node.x < x + x_size:
				if node.y > y and node.y < y + y_size:
					return False
		
		return True
	
	def check_collision(self, node, parent, obsticle_list):
		x_step = (node.x - parent.x) / obsticle_detection_resolution
		y_step = (node.y - parent.y) / obsticle_detection_resolution
		
		for i in range(obsticle_detection_resolution+1):
			x = parent.x + (i * x_step)
			y = parent.y + (i * y_step)
			if not self.check_point_collision(Node(x, y), obsticle_list):
				# pg.draw.circle(surf, RED, [x, height-y], 2)
				return False
			# pg.draw.circle(surf, GREEN, [x, height-y], 2)

		return True
	
	def get_neighbours(self, node, node_list, distance):
		neighbours = []
		for n in node_list:
			delta_x = n.x - node.x
			delta_y = n.y - node.y

			if delta_x == 0 and delta_y == 0:
				continue
			
			if abs(delta_x) < distance and abs(delta_y) < distance:
				neighbours.append(n)

		return neighbours
	
	def find_nearest(self, node, node_list):
		nearest = None
		for n in node_list:
			if nearest == None:
				nearest = n
			
			if math.sqrt((n.x - node.x)**2 + (n.y - node.y)**2) < math.sqrt((nearest.x - node.x)**2 + (nearest.y - node.y)**2):
				nearest = n
		
		return nearest
	
	def steer(self, node, nearest):
		delta_x = node.x - nearest.x
		delta_y = node.y - nearest.y
		distance = math.sqrt(delta_x**2 + delta_y**2)
		if distance < node_max_distance:
			return [node.x, node.y]
		angle = math.atan2(delta_y, delta_x)
		new_x = nearest.x + (node_max_distance * math.cos(angle))
		new_y = nearest.y + (node_max_distance * math.sin(angle))
		return [new_x, new_y]

	def find_low_cost(self, node, node_list, obsticle_list):
		cost_list = []

		if not node_list:
			print("no nodes")
			return None

		for n in node_list:
			if self.check_collision(node, n, obsticle_list):
				distance = math.sqrt((n.x - node.x)**2 + (n.y - node.y)**2)
				cost_list.append(n.cost + distance)
			else:
				cost_list.append(float("inf"))
		
		min_cost = min(cost_list)

		if min_cost == float("inf"):
			# print("min_cost is inf")
			return None

		return node_list[cost_list.index(min_cost)]

	def check_end(self, node, obsticle_list):
		distance = math.sqrt((self.end.x - node.x)**2 + (self.end.y - node.y)**2)
		if distance < node_neighbour_distance:
			if self.check_collision(Node(self.end.x, self.end.y), node, obsticle_list):
				return True
			else:
				return False
		else:
			return False
		
	def remove_leaf(self):
		for node in nodes:
			if len(node.children) == 0:
				node.parent.children.remove(node)
				nodes.remove(node)

	def rewire(self, test_nodes, node_list, obsticle_list):
		for node in test_nodes:
			if (node.x == self.end.x and node.y == self.end.y) or (node.x == self.start.x and node.y == self.start.y):
				continue
			neighbours = self.get_neighbours(node, node_list, node_neighbour_distance)
			low_cost_node = self.find_low_cost(node, neighbours, obsticle_list)
			if low_cost_node:
				if not low_cost_node.parent == None:
					node.add_parent(low_cost_node)
				elif low_cost_node.parent == None and not (low_cost_node.x == self.start.x and low_cost_node.y == self.start.y):
					self.remove_children(low_cost_node)
	
	def find_end_node(self, node_list):
		for node in node_list:
			if node.x == self.end.x and node.y == self.end.y:
				return node
			else:
				return None
	
	def remove_from_collision(self, node_list, obsticle_list):
		global path
		for node in node_list:
			if not self.check_point_collision(node, obsticle_list):
				node.parent.children.discard(node)
				nodes.remove(node)
	
	def remove_children(self, node):
		for child in node.children:
			if len(child.children) > 0:
				for child_child in child.children:
					self.remove_children(child_child)
			else:
				if child in nodes:
					nodes.remove(child)


############################################################################################################


# creates drone
drone = Drone(100, 100, 0, 0)
nodes.append(Node(drone.x, drone.y, None, 0))


# creates obsticle drone
obsticle_drone = Drone(100, 400, 0, 0)
dynamic_obsticles = []
# dynamic_obsticles.append(obsticle_drone)


# creates obsticles randomly
static_obsticles = []
for i in range(10):
	x = random.randint(200, width-50)
	y = random.randint(50, height-50)
	size_x = random.randint(50, 100)
	size_y = random.randint(50, 300)
	static_obsticles.append(pg.Rect(x, y, size_x, size_y))


############################################################################################################


def find_path(start, end):
	global path
	rrt = DynamicRRT(start, end, [width, height])
	while(1):
		if limitFPS and len(path) > 0:
			''' Limit FPS '''
			clock.tick(limit)
		else:
			clock.tick()

		surf.fill(GRAY)
		pg.draw.circle(surf, BLUE, [rrt.start.x, height-rrt.start.y], 10)
		pg.draw.circle(surf, RED, [rrt.end.x, height-rrt.end.y], 10)

		for event in pg.event.get():
			''' Exit on X '''
			if event.type == pg.QUIT:
				pg.quit()
				sys.exit()

			if event.type == pg.MOUSEBUTTONDOWN:
				''' Left click '''
				if event.button == 1:
					return

				''' Right click '''
				if event.button == 3:
					coords = pg.mouse.get_pos()
					dynamic_obsticles.append(pg.Rect(coords[0] - 25, height - coords[1] - 25, 50, 50))
					near_node = rrt.find_nearest(Node(coords[0] - 25, height - coords[1] - 25), nodes)
					neighbour_nodes = rrt.get_neighbours(near_node, nodes, node_neighbour_distance)
					rrt.remove_from_collision(neighbour_nodes, static_obsticles + dynamic_obsticles)
					neighbour_nodes = rrt.get_neighbours(near_node, nodes, node_neighbour_distance*3)
					rrt.rewire(neighbour_nodes, nodes, static_obsticles + dynamic_obsticles)

		random_point = rrt.get_random_point()
		nearest = rrt.find_nearest(Node(random_point[0], random_point[1]), nodes)
		# pg.draw.circle(surf, RED, [random_point[0], height-random_point[1]], 2)

		if abs(random_point[0] - nearest.x) < node_min_distance and abs(random_point[1] - nearest.y) < node_min_distance:
			continue
		
		new_point = rrt.steer(Node(random_point[0], random_point[1]), nearest)
		neighbours = rrt.get_neighbours(Node(new_point[0], new_point[1]), nodes, node_neighbour_distance)
		low_cost_node = rrt.find_low_cost(Node(new_point[0], new_point[1]), neighbours, static_obsticles + dynamic_obsticles)
		if low_cost_node:
			new_node = Node(new_point[0], new_point[1])
			new_node.add_parent(low_cost_node)
			nodes.append(new_node)

			if rrt.check_end(new_node, static_obsticles + dynamic_obsticles):
				neighbours = rrt.get_neighbours(rrt.end, nodes, node_neighbour_distance)
				low_cost_node = rrt.find_low_cost(rrt.end, neighbours, static_obsticles + dynamic_obsticles)
				if low_cost_node:
					rrt.end.add_parent(low_cost_node)
				if rrt.end not in nodes:
					nodes.append(rrt.end)
				path = rrt.end.path()
			
			if len(nodes) > rrt.maxIter:
				rrt.remove_leaf()
			rrt.rewire(neighbours, nodes, static_obsticles + dynamic_obsticles)
			draw_shit()
			# time.sleep(10)


def draw_shit():
	# surf.fill(GRAY)
	draw_static_obstacles()
	draw_dynamic_obstacles()
	for node in nodes:
		pg.draw.circle(surf, BLUE, [node.x, height-node.y], 2)

		if node.parent == None:
			continue

		pg.draw.line(surf, BLACK, [node.parent.x, height-node.parent.y], [node.x, height-node.y])

	if len(path) > 1:
		draw_path(path)
	
	pg.display.flip()


def draw_static_obstacles():
	for obs in static_obsticles:
		pg.draw.rect(surf, BLACK, pg.Rect(obs[0], height-obs[1]-obs[3], obs[2], obs[3]))


def draw_dynamic_obstacles():
	for obs in dynamic_obsticles:
		pg.draw.rect(surf, BLACK, pg.Rect(obs[0], height-obs[1]-obs[3], obs[2], obs[3]))


def draw_path(path):
	for node in path:
		if node.parent != None:
			pg.draw.line(surf, GREEN, (node.x, height-node.y), (node.parent.x, height-node.parent.y), width=2)


def refresh():
	global path, nodes
	path = []
	nodes = []
	nodes.append(Node(drone.x, drone.y, None, 0))
	surf.fill(GRAY)
	draw_static_obstacles()

''' Moves the drone following the path '''
def fly_path():
	global path, nodes
	if math.sqrt((path[0].x - drone.x)**2 + (path[0].y - drone.y)**2) < jitter:
		
		'''Moves the drone in the real world'''
		if len(path) > 1:
			dist_x = path[1].x - drone.x
			dist_y = path[1].y - drone.y
			dist_r = math.sqrt(dist_x**2 + dist_y**2)
			rot_rad = math.atan2(dist_y, dist_x)
			#abbsolute angle
			abs_rot_deg = math.degrees(rot_rad)
			
			#correct with drones current heading in relation to the absolute angle
			rot_deg	= abs_rot_deg - drone.dir
			drone.dir += rot_deg

			print(rot_deg)
			
			""" tello.rotate_clockwise(int(rot_deg))
			if dist_r > 20:
				tello.move_forward(int(dist_r)) """
			""" tello.go_xyz_speed(int(path[0].x), int(path[0].y), 0, 15) """

			""" acc_x = tello.get_acceleration_x()
			print(acc_x)
			print(tello.get_acceleration_y())
			print(tello.get_acceleration_z())
			"""
		path.pop(0)
	
	if len(path) == 0:
		path = []
		nodes = []
		""" nodes.append(Node("drone", x=drone.x, y=drone.y)) """
		return
  
	drone.move((path[0].x, path[0].y))


############################################################################################################

target = []
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
				refresh()
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target = [coords[0], height-coords[1]]
				pg.draw.circle(surf, BLUE, coords, 5)
				find_path([drone.x, drone.y], [coords[0], height-coords[1]])
				print("--- %s seconds ---" % (time.time() - start_time))

			''' Right click '''
			if event.button == 3:
				coords = pg.mouse.get_pos()
				dynamic_obsticles.append(pg.Rect(coords[0] - 25, height - coords[1] - 25, 50, 50))
				find_path([drone.x, drone.y], [target[0], target[1]])
		
		''' Buttons '''
		if event.type == pg.KEYDOWN:
			''' Down arrow == land '''
			if event.key == pg.K_DOWN:
				tello.land()
			''' Up arrow == takeoff '''
			if event.key == pg.K_UP:
				tello.takeoff()
			''' Enter == connect '''
			if event.key == pg.K_RETURN:
				tello.connect()
			''' W key == forward '''
			if event.key == pg.K_w:
				tello.move_forward(20)
			''' A key == left '''
			if event.key == pg.K_a:
				tello.move_left(20)
			''' S key == backawrds '''
			if event.key == pg.K_s:
				tello.move_back(20)
			''' D key == right '''
			if event.key == pg.K_d:
				tello.move_right(20)
			''' Spacebar == emergency '''
			if event.key == pg.K_SPACE:
				tello.emergency()

	surf.fill(GRAY)

	draw_static_obstacles()
	draw_shit()

	if path:
		draw_path(path)
		fly_path()
	pg.draw.circle(surf, BLUE, [drone.x, height-drone.y], 10)
	pg.display.flip()