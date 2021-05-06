#!/usr/bin/env python3
import rospy
import math
import random
import numpy as np 

from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist


class RRT:
	"""RRT class that performs Path Planning (Rapidly Exploring Random Trees)"""
	class Node:
		"""Node class to express different nodes in the trees"""
		def __init__(self, x, y):
			"""initialize parameters of a node"""
			self.x = x
			self.y = y
			self.path_x = []
			self.path_y = []
			self.parent = None

	def __init__(self, start, goal, costmap, min_max_points, width,
				 max_len = 3.0,
				 path_resolution = 0.5,
				 goal_sample_rate = 5,
				 max_iter = 20000):
		"""initialize the parameters of the RRT class"""
		self.start = self.Node(int(start[0]), int(start[1]))
		self.end = self.Node(int(goal[0]), int(goal[1]))
		self.min_point = min_max_points[0]  # min point in the graph
		self.max_point = min_max_points[1]  # max point in the graph
		self.width = width
		self.costmap = costmap  # list of all the obstacles
		self.max_len = max_len  # max distance to travel from a node
		self.path_resolution = path_resolution  # path resolution
		self.goal_sample_rate = goal_sample_rate  # range within which the goal node will be considered
		self.max_iter = max_iter  # maximum number of times to run the loop for rrt
		self.node_list = []  # list for adding all the valid nodes

	def find_path(self):
		"""Function the find the path between start and end positions"""
		self.node_list = [self.start]

		for i in range(self.max_iter):
			random_node = self.get_random_node()
			nearest_node_ind = self.get_nearest_node_index(self.node_list, random_node) 
			nearest_node = self.node_list[nearest_node_ind] 

			new_node = self.move(nearest_node, random_node, self.max_len)

			if self.check_collision(new_node, self.costmap):  
				self.node_list.append(new_node)

			if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.max_len:   
				final_node = self.move(self.node_list[-1], self.end, self.max_len) 
				if self.check_collision(final_node, self.costmap):
					return self.generate_final_course(len(self.node_list) - 1)

		return None

	def move(self, from_node, to_node, max_len = float("inf")):
		"""Function to move from one node to another"""
		new_node = self.Node(from_node.x, from_node.y)
		d, angle = self.calc_dist_and_angle(new_node, to_node) 

		new_node.path_x = [new_node.x]
		new_node.path_y = [new_node.y]

		if d < max_len:
			max_len = d

		n_expand = math.floor(max_len / self.path_resolution)

		for _ in range(n_expand):
			# moving in steps of length path_resolution with the anlge
			new_node.x += self.path_resolution * math.cos(angle)
			new_node.y += self.path_resolution * math.sin(angle) 

			# appending the point the path of the node
			new_node.path_x.append(new_node.x)
			new_node.path_y.append(new_node.y)

		d, _ = self.calc_dist_and_angle(new_node, to_node)

		if d <= self.path_resolution:
			new_node.path_x.append(to_node.x)
			new_node.path_y.append(to_node.y)

			new_node.x = to_node.x
			new_node.y = to_node.y

		new_node.parent = from_node

		return new_node

	def generate_final_course(self, goal_ind):
		"""This Function generates the final path between start and goal"""
		path = [[int(self.end.x), int(self.end.y)]]  # first appending the goal to the path
		node = self.node_list[goal_ind]

		while node.parent is not None:  
			path.append([int(node.x), int(node.y)])
			node = node.parent

		path.append([int(node.x), int(node.y)])

		return path

	def calc_dist_to_goal(self, x, y):
		"""Function to calculate the distance between current node and goal"""
		dx = x - self.end.x
		dy = y - self.end.y 

		return math.hypot(dx, dy)

	def get_random_node(self):
		"""Function to generate a random node"""
		x, y = self.index_to_xy(int(random.uniform(self.min_point, self.max_point)))
		return self.Node(x, y)

	def index_to_xy(self, index):
		return index % self.width, math.floor(index / self.width)

	def xy_to_index(self, x, y):
		return y * self.width + x

	def check_collision(self, node, costmap):
		"""Function to check if the node collides with an obstacle"""
		if node is None:
			return False 

		ind = self.xy_to_index(int(node.x), int(node.y))

		if costmap[ind] == 0:
			return True
		return False

	@staticmethod
	def get_nearest_node_index(node_list, random_node):
		"""Function to find the index of the node nearest to the random_node"""
		dlist = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 for node in node_list]
		return dlist.index(min(dlist))


	@staticmethod
	def calc_dist_and_angle(from_node, to_node):
		"""Calculate distance between 2 nodes"""
		dx = to_node.x - from_node.x
		dy = to_node.y - from_node.y

		return math.hypot(dx, dy), math.atan2(dy, dx)

def clean_shutdown():
	cmd_vel.publish(Twist())
	rospy.sleep(1)

def make_plan(req):
	rospy.loginfo("Got a request")
	costmap = req.costmap_ros
	width = req.width
	start = req.start
	goal = req.goal

	path_x = []
	path_y = []

	start_time = rospy.Time.now()

	# set initial parameters
	rrt = RRT(
		start = start,
		goal = goal,
		min_max_points = [0, len(costmap)],
		costmap = costmap,
		width = width)

	# planning the path
	path = rrt.find_path()

	if not path:
		rospy.logwarn("No path found")
	else:
		execution_time = rospy.Time.now() - start_time
		rospy.loginfo("Total execution time : %s seconds", str(execution_time.to_sec()))

		for xy in path:
			path_x.append(xy[0])
			path_y.append(xy[1])

		path_x = path_x[::-1]
		path_y = path_y[::-1]

	resp = PathPlanningPluginResponse()
	resp.plan_x = path_x
	resp.plan_y = path_y
	return resp


if __name__ == "__main__":
	rospy.init_node("rrt_path_planning_service_server", log_level = rospy.INFO, anonymous = False)
	make_plan_service = rospy.Service("/move_base/PPPlugin/make_plan", PathPlanningPlugin, make_plan)
	cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
	rospy.on_shutdown(clean_shutdown)

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)
	rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot = True)
