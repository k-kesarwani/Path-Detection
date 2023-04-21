# Required: https://github.com/Bharath2/Informed-RRT-star

import pygame as pg
import random
import sys, math, time


''' Window prefs '''
width = 1080				 # Window width
height = 1080				# Window height
pg.init()
surf = pg.display.set_mode((width, height), pg.RESIZABLE)
pg.display.set_caption("IRRT* Test")
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
drone_size = 10					# size of the drone collision


''' RRT* Settings '''
max_iterations = 600
max_node_distance = 50


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


##################################################################################################


# creates drone
drone = Drone(100, 100, 0, 0)


##################################################################################################


import numpy as np
from PathPlanning import RRT, RRTStar, Map


# creates obsticles randomly
obstacles = []
obss = []
for i in range(40):
	x = random.randint(150, width-50)
	y = random.randint(50, height-50)
	size_x = random.randint(50, 50)
	size_y = random.randint(50, 50)
	obstacles.append([x, y, x+size_x, y+size_y])
	obss.append(pg.Rect(x, height - y - size_y, size_x, size_y))

#initialise environment map
bounds = np.array([0,width])
mapobs = Map(obstacles,bounds,dim = 2, path_resolution=5)


'''Global variables '''
rrt = None
path = np.array([[]])


############################################################################################################


def draw_obstacles():
	for obs in obss:
		pg.draw.rect(surf, RED, obs)
		pg.draw.rect(surf, BLACK, pg.Rect(obs.left + drone.size, obs.top + drone.size, obs.width - (2*drone.size), obs.height - (2*drone.size)))


def draw_path():
	for i in range(len(path)-1):
		pg.draw.line(surf, GREEN, (path[i, 0], height - path[i, 1]), (path[i+1, 0], height - path[i+1, 1]), width=4)

def draw_nodes():
	'''plot the whole graph'''
	for node in rrt.tree.all():
		if node.parent:
			xy = np.c_[node.p,node.parent.p]
			pg.draw.line(surf, (0, 0, 0), (xy[0, 0], height - xy[1, 0]), (xy[0, 1], height - xy[1, 1]), width=4)


''' Moves the drone following the path '''
def fly_path():
	global path, nodes
	if math.sqrt((path[0, 0] - drone.x)**2 + (path[0, 1] - drone.y)**2) < jitter:
		path = np.delete(path, 0, 0)
	
	if not path.any():
		path = np.array([[]])
		return

	drone.move((path[0, 0], path[0, 1]))


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
				path = np.array([[]])
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = [coords[0], height-coords[1]]
				#initialise RRT
				rrt = RRTStar(start=np.array([drone.x, drone.y]), goal=np.array([target_node[0], target_node[1]]), Map=mapobs, max_iter=max_iterations, max_extend_length=max_node_distance, goal_sample_rate=0.1)
				#plan path
				path, _ = rrt.plan()
				print("--- %s seconds ---" % (time.time() - start_time))

			''' Middle click '''
			if event.button == 2:
				pass

			''' Right click '''
			if event.button == 3:
				path = np.array([[]])
				start_time = time.time()
				# records the click position
				coords = pg.mouse.get_pos()
				target_node = [coords[0], height-coords[1]]
				#initialise RRT
				rrt = RRT(start=np.array([drone.x, drone.y]), goal=np.array([target_node[0], target_node[1]]), Map=mapobs, max_iter=max_iterations, max_extend_length=max_node_distance)
				#plan path
				path = rrt.plan()
				print("--- %s seconds ---" % (time.time() - start_time))

	surf.fill(GRAY)

	if type(path) != np.ndarray:
		print("Did not find any path")
		path = np.array([[]])
	elif path.any():
		pg.draw.circle(surf, YELLOW, [target_node[0], height - target_node[1]], 10)
		fly_path()
		draw_nodes()
		draw_path()
	
	draw_obstacles()
	pg.draw.circle(surf, BLUE, [drone.x, height - drone.y], 10)
	pg.display.flip()