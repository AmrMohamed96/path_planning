#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Int32, Byte, String

import numpy as np
import math
from heapq import heappush, heappop
# static parameter
robot_num = 4
calc_flag = 0

# robot position list (start postition)
robot_position = [0,0]
robot_pos_cm = [0, 0]

# goal point variables (in squares)
goal_position = [0,0]

# map dimenstions
m = 10
n = 10
square_size = 17.5

# map list
the_map = []

# representation numbers
obst_rep = 1
rob_rep = 2
route_rep = 3   
finish_rep = 4

# obstacles positions list
obstacles_positions = [ [0,0], [0,0], [0,0], [0,0], [0,0] ]
obst_pos_cm = [ [0,0], [0,0], [0,0], [0,0], [0,0]]

# extra blocks for robots
redundant_blocks = 1
red_dir = 8
red_x = [redundant_blocks, redundant_blocks, 0, -redundant_blocks, -redundant_blocks, -redundant_blocks, 0, redundant_blocks]
red_y = [0, redundant_blocks, redundant_blocks, redundant_blocks, 0, -redundant_blocks, -redundant_blocks, -redundant_blocks]

# possible movements
dirs = 8
dx = [1, 1, 0, -1, -1, -1, 0, 1]
dy = [0, 1, 1, 1, 0, -1, -1, -1]

def discretize_positions():
	"""
	Discretize the given positions with cm into a grid with
	predefined square size
	"""
	robot_position[0] = int( round( robot_pos_cm[0] / square_size ) )
	robot_position[1] = int( round( robot_pos_cm[1] / square_size ) )

	for i in range( len(obst_pos_cm) ):
		obstacles_positions[i][0] = int( round( obst_pos_cm[i][0] / square_size ) )
		obstacles_positions[i][1] = int( round( obst_pos_cm[i][1] / square_size ) )

def check_validity():
	"""
	Check that all positions are already within the correct boundaries
	of the system
	"""
	# check that all positions are within the grid boundaries
	# check that it is not on the reference
	if not ( (robot_position[0] == 0 and robot_position[1] == 0 ) ):
		if not ( ( 0 <= robot_position[0] <= m ) and ( 0 <= robot_position[1] <= n) ):
			print ("Robot position is not within the grid")
			return 0

		for i in range ( len(obstacles_positions) ):
			if not ( ( obstacles_positions[i][0] == 0 ) and (obstacles_positions[i][1] == 0 ) ):
				if not ( ( 0 < obstacles_positions[i][0] <= m ) and ( 0 < obstacles_positions[i][1] <= n) ):
					print "Some positions are not within the grid"
					return 0
			else:
				print "Obstacle is on reference"
				return 0

		if not ( ( 0 <= goal_position[0] <= m) and ( 0 <= goal_position[1] <= n) ):
			print "Goal is not within the map"
			return 0
		# returns 1 if successful
		return 1
	else:
		print "Robot is on reference"
		return 0

def create_map():
	"""
	Creates a map based on discretized positions
	Returns zero if it fails to do so, 1 if it succeeds
	"""
	global the_map
	# discretize the positions into squares
	discretize_positions()

	# primary check: everything is within the acceptable grid
	if check_validity():
		if goal_position[0] or goal_position[1]:
			# create an empty array representing the map
			the_map = np.zeros( (n,m) )

			# allocate obstacles positions on the map
			for i in range ( len(obstacles_positions) ):
				the_map[obstacles_positions[i][1]][obstacles_positions[i][0]] = obst_rep

				# add redundant blocks for each obstacles in defined directions based on required redundancy size
				for j in range ( red_dir ):
					if (0 < obstacles_positions[i][1] + red_x[j] <= m-1 ) and ( 0 < obstacles_positions[i][0] + red_y[j] <= n-1):
						the_map[obstacles_positions[i][1] + red_y[j] ][obstacles_positions[i][0] + red_y[j] ] = obst_rep


			# check the robot is not on an obstacle then allocate point
			if the_map[robot_position[1]][robot_position[0]] == obst_rep:
				print "Robot is on an obstacle."
				return 0
			else:
				the_map[robot_position[1]][robot_position[0]] == rob_rep
				return 1

			# checks that the goal point is not set on an obstacle
			if the_map[goal_position[1]][goal_position[0]] == obst_rep:
				print "Final goal is on an obstacle."
				return 0
			else:
				the_map[goal_position[1]][goal_position[0]] = finish_rep
				return 1

		else:
			print "No goal point has been set"
	else:
		return 0

class node:
	xPos = 0
	yPos = 0
	G = 0
	F = 0

	def __init__(self, xPos, yPos, G, F):
		self.xPos = xPos
		self.yPos = yPos
		self.G = G
		self.F = F

	def __lt__(self, other):
		return self.F < other.F

	def updateF(self, xEnd, yEnd):
		self.F = self.G + self.H(xEnd, yEnd) * 10

	def H(self, xEnd, yEnd):
		xd = xEnd - self.xPos
		yd = yEnd - self.yPos
		d = math.sqrt(xd*xd + yd*yd)
		return(d)

	def updateG(self, dirs, d):
		if dirs == 8 and d % 2 != 0:
			self.G = self.G + 14
		else:
			self.G = self.G +10

def pathPlanner(the_map, n, m, dirs, deltX, deltaY, xA, yA, xB, yB):
	closed_nodes = []
	open_nodes = []
	parent_dir = []
	row = [0] * n
	for i in range(m):
		closed_nodes.append(list(row))
		open_nodes.append(list(row))
		parent_dir.append(list(row))

	pq = [[], []]
	pqi = 0

	nA = node(xA, yA, 0, 0)
	nA.updateF(xB, yB)
	heappush (pq[pqi], nA)
	open_nodes[yA][xA] = nA.F

	while len(pq[pqi]) > 0:

		n1 = pq[pqi][0]
		nA = node(n1.xPos, n1.yPos, n1.G, n1.F)
		x = nA.xPos
		y = nA.yPos
		heappop(pq[pqi])
		open_nodes[y][x] = 0
		closed_nodes[y][x] = 1

		if x == xB and y ==  yB:
			path = ''

			while not (x == xA and y == yA):
				j = parent_dir[y][x]
				c = str((j + dirs/2) % dirs)
				path = c + path
				x = x + dx[j]
				y = y + dy[j]

			return path

		for i in range(dirs):
			xdx = x + dx[i]
			ydy = y + dy[i]

			if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m-1 or the_map[ydy][xdx] == 1
					or closed_nodes[ydy][xdx] == 1):

				m0 = node(xdx, ydy, nA.G, nA.F)
				m0.updateG(dirs, i)
				m0.updateF(xB, yB)

				if open_nodes[ydy][xdx] == 0:
					open_nodes[ydy][xdx] = m0.F
					heappush(pq[pqi], m0)
					parent_dir[ydy][xdx] = (i + dirs/2) % dirs

				elif open_nodes[ydy][xdx] > m0.F:
					open_nodes[ydy][xdx] = m0.F
					parent_dir[ydy][xdx] = (i + dirs/2) % dirs

					while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
						heappush(pq[1-pqi], pq[pqi][0])
						heappop(pq[pqi])
					heappop(pq[pqi])

					if len(pq[pqi]) > len(pq[1-pqi]):
						pqi = 1 - pqi
					while len(pq[pqi]) > 0:
						heappush(pq[1-pqi], pq[pqi][0])
						heappop(pq[pqi])
					pqi = 1 - pqi
					heappush(pq[pqi], m0)

	return 'Planning Failed'

def update_map_w_route(route):
	global the_map
	if len(route) > 0:
		x = robot_position[0]
		y = robot_position[1]
		the_map[y][x] = 2
		for i in range(len(route)):
			j = int(route[i])
			x += dx[j]
			y += dy[j]
			the_map[y][x] = 3
			the_map[goal_position[1]][goal_position[0]]=4

def get_path_squares(map):
	goals_list = []
	# extracting path points from the map
	for y in range(m):
		for x in range(n):
			xy = the_map[y][x]
			if  xy == 3 or xy ==4:
				goals_list.append(x)
				goals_list.append(y)

	# arranging the extracted list to start from the robot position
	rows = len(goals_list) /2
	reshaped_goals_list = np.reshape( goals_list, (rows,2))
	last_point = [robot_position[0], robot_position[1]]
	arranged_goals_list = []

	while ( last_point[0] != goal_position[0] ) or ( last_point[1] != goal_position[1]):
		for i in range ( len(reshaped_goals_list) ):
			if ( reshaped_goals_list[i][0] != -11 ) and ( reshaped_goals_list[i][1] != -11):
				error_x = abs (last_point[0] - reshaped_goals_list[i][0])
				error_y = abs (last_point[1] - reshaped_goals_list[i][1])

				if (error_x == 0 and error_y == 0)or(error_x == 0 and error_y == 1) or (error_x == 1 and error_y == 0) or (error_x == 1 and error_y == 1):
					arranged_goals_list.append ( [ reshaped_goals_list[i][0], reshaped_goals_list[i][1] ])
					last_point = [reshaped_goals_list[i][0], reshaped_goals_list[i][1]]
					reshaped_goals_list[i] = [-11, -11]
					break
	return arranged_goals_list

def path_planning_routine():
	map_valid = create_map()
	if map_valid:
		route = pathPlanner(the_map,n,m,dirs,dx,dy,robot_position[0], robot_position[1],goal_position[0],goal_position[1])
		update_map_w_route(route)
		path_squares = get_path_squares(the_map)
		
		print "Path is: \n", path_squares, "\n"
		print "Final Map is: \n", the_map, "\n"
		publisher_path = np.array(path_squares).flatten().tolist()
		if ( len(publisher_path) <= 0):
			publisher_path = robot_position
		pathPublisher.publish(Int32MultiArray(data=publisher_path))
		return path_squares
	else:
		print "Map is invalid. No Planning will occur"
		return 0

# subscribers callback functions
def robot_position_callback(data):
	global robot_pos_cm
	robot_pos_cm[0] = data.data[0]
	robot_pos_cm[1] = data.data[1]

def obst1_position_callback(data):
	global obst_pos_cm
	obst_pos_cm[0][0] = data.data[0] * 17.5
	obst_pos_cm[0][1] = data.data[1] * 17.5

def obst2_position_callback(data):
	global obst_pos_cm
	obst_pos_cm[1][0] = data.data[0] * 17.5
	obst_pos_cm[1][1] = data.data[1] * 17.5

def obst3_position_callback(data):
	global obst_pos_cm
	obst_pos_cm[2][0] = data.data[0]
	obst_pos_cm[2][1] = data.data[1]

def obst4_position_callback(data):
	global obst_pos_cm
	obst_pos_cm[3][0] = data.data[0]
	obst_pos_cm[3][1] = data.data[1]

def obst5_position_callback(data):
	global obst_pos_cm
	obst_pos_cm[4][0] = data.data[0]
	obst_pos_cm[4][1] = data.data[1]

def goal_point_callback(data):
	global goal_position
	goal_position[0] = data.data[(2 * robot_num) - 2]
	goal_position[1] = data.data[(2 * robot_num) - 1]

def start_routine_callback(data):
	global calc_flag
	if data.data == robot_num and calc_flag ==0:
		path_planning_routine()
		calc_flag = 1

if __name__ == '__main__':
	# initializing the ros node
	rospy.init_node('path_planning_rob4')
	rospy.loginfo('%s started' % rospy.get_name())

	# initialize position subscribers
	# robot current position = start point
	rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, robot_position_callback)

	# global obstacles positions
	rospy.Subscriber('obst1', Int32MultiArray, obst1_position_callback)
	rospy.Subscriber('obst2', Int32MultiArray, obst2_position_callback)

	# relative obstacles positions
	rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, obst3_position_callback)
	rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, obst4_position_callback)
	rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, obst5_position_callback)

	# initialize goal subscriber
	rospy.Subscriber('next_goals_px', Int32MultiArray, goal_point_callback)

	# path planning flag to be polled 
	rospy.Subscriber('robot_to_be_moved', Int32, start_routine_callback)

	# initialize path publisher
	pathPublisher = rospy.Publisher('Planning_Output4', Int32MultiArray, queue_size=5)

	# current map publisher
	currentMap = rospy.Publisher('current_map_rob4', Int32MultiArray, queue_size=1)

	# wait time to make sure that all connections are made
	rospy.sleep(5)

	while not rospy.is_shutdown():
		rospy.spin()
