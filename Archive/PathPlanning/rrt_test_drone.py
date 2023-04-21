'''
Requirements:
pip3 install pygame anytree djitellopy
'''

import pygame as pg
import random
import sys, math, time
import time

from anytree import Node
from djitellopy import Tello


''' Window prefs '''
width = 300		 # Window width
height = 300				# Window height
pg.init()
surf = pg.display.set_mode((width, height), pg.RESIZABLE)
pg.display.set_caption("RRT Test")
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

''' Drone simulation settings '''
jitter = 3
easing = 0.2
max_speed = 6
acceleration = 1
distance = 100

''' Drone settings '''
tello = Tello()

''' RRT settings '''
nodes = []
path = []
node_max_distance = 60
node_min_distance = 20


############################################################################################################


class Drone():
	
	def __init__(self, x, y, xv, yv, acc=acceleration, easing=easing, max_speed=max_speed, size=10, dir=0):
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
		
		# getting the delta_r
		theta = math.atan2(delta_y, delta_x)
		
		# adding acceleration
		if math.sqrt(self.xv**2 + self.yv**2) < self.max_speed:
			self.xv += self.acc * math.cos(theta)
			self.yv += self.acc * math.sin(theta)

		# decelerating
		if delta_r < distance or not (math.atan2(self.yv, self.xv) < theta+0.3 and math.atan2(self.yv, self.xv) > theta-0.3):
			self.xv *= 1 - self.easing
			self.yv *= 1 - self.easing

		# moving the drone
		self.x += self.xv
		self.y += self.yv


############################################################################################################


# creates drone
drone = Drone(100, 100, 0, 0)
coords = [drone.x, drone.y]
nodes.append(Node("drone", x=drone.x, y=drone.y))


# creates obsticles randomly
obsticles = []
for i in range(30):
	x = random.randint(200, width-50)
	y = random.randint(50, height-50)
	size_x = random.randint(50, 100)
	size_y = random.randint(50, 300)
	obsticles.append(pg.Rect(x, y, size_x, size_y))


############################################################################################################


''' Creates a node in random direction '''
def create_node():
	random_x = random.randint(30, width-30)
	random_y = random.randint(30, height-30)

	closest_node = nodes[0]

	# checks which of the nodes is closest
	for node in nodes:
		if node.name != "target":
			if math.sqrt((closest_node.x - random_x)**2 + (closest_node.y - random_y)**2) > math.sqrt((node.x - random_x)**2 + (node.y - random_y)**2):
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
	for i in range(6):
		test_x += 10 * math.cos(theta)
		test_y += 10 * math.sin(theta)
		
		for obs in obsticles:
			if (obs.collidepoint(test_x, test_y) or obs.collidepoint(test_x+drone.size, test_y) 
												or obs.collidepoint(test_x, test_y+drone.size)
												or obs.collidepoint(test_x+drone.size, test_y+drone.size)
												or obs.collidepoint(test_x-drone.size, test_y)
												or obs.collidepoint(test_x, test_y-drone.size)
												or obs.collidepoint(test_x-drone.size, test_y-drone.size)):
				return None

	if delta_r > node_max_distance:
		x = (node_max_distance * math.cos(theta)) + closest_node.x
		y = (node_max_distance * math.sin(theta)) + closest_node.y
	elif delta_r < node_min_distance:
		return None
	else:
		x = random_x
		y = random_y

	nodes.append(Node("child", parent=closest_node, x=x, y=y))
	return nodes[-1]


''' Finds path to the target '''
def find_path():
	while True:
		node = create_node()

		if node == None:
			continue
	
		if math.sqrt((nodes[1].x - node.x)**2 + (nodes[1].y - node.y)**2) < node_max_distance:
			nodes[1].parent = node
			return
		
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
	loc_x = 100
	loc_y = 100
	if math.sqrt((path[0].x - drone.x)**2 + (path[0].y - drone.y)**2) < jitter:

		'''Moves the drone in the real world'''
		dist_x = path[0].x - loc_x
		dist_y = path[0].y - loc_y
		dist_r = math.sqrt(dist_x**2 + dist_y**2)
		rot_rad = math.atan2(dist_y, dist_x)
		rot_deg = (rot_rad * 180) / math.pi

		loc_x = path[0].x
		loc_y = path[0].y

		tello.rotate_clockwise(int(rot_deg))
		if dist_r > 20:
			""" tello.move_forward(int(dist_r)) """
		""" tello.go_xyz_speed(int(path[0].x), int(path[0].y), 0, 15) """

		acc_x = tello.get_acceleration_x()
		print(acc_x)
		print(tello.get_acceleration_y())
		print(tello.get_acceleration_z())

		path.pop(0)
	
	if len(path) == 0:
		path = []
		nodes = []
		nodes.append(Node("drone", x=drone.x, y=drone.y))
		return
  
	drone.move((path[0].x, path[0].y))



############################################################################################################


target_node = None
while True:
	if limitFPS:
		''' Limit FPS '''
		clock.tick(limit)
		fps = clock.get_fps()
		print("	   " + str(int(fps)) + "	   ", end='\r')
	else:
		clock.tick()
		fps = clock.get_fps()
		print("	   " + str(int(fps)) + "	   ", end='\r')

	for event in pg.event.get():
		''' Exit on X '''
		if event.type == pg.QUIT:
			pg.quit()
			sys.exit()

		''' Mouse Buttons '''
		if event.type == pg.MOUSEBUTTONDOWN:
			''' Left click '''
			if event.button == 1:
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = Node("target", x=coords[0], y=coords[1])
				nodes.append(target_node)
				find_path()
				path = list(target_node.path)

			''' Middle click '''
			if event.button == 2:
				nodes = []
				nodes.append(Node("drone", x=drone.x, y=drone.y))
				path = []

			''' Right click '''
			if event.button == 3:
				pass
		
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

	show_nodes()

	if len(path) > 0:
		pg.draw.circle(surf, YELLOW, [target_node.x, target_node.y], 10)
		show_path()
		fly_path()

	for obs in obsticles:
		pg.draw.rect(surf, BLACK, obs)

	pg.draw.circle(surf, RED, [drone.x, drone.y], drone.size)
	pg.display.flip()