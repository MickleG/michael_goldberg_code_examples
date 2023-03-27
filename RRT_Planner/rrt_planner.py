from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import math
import os
import sys
import random

#All constants pulled from UR5 technical manual

UR5_JOINT_INDICES = [0, 1, 2]
JOINT_RANGES = [[-2 * np.pi, 2 * np.pi], [-2 * np.pi, 2 * np.pi], [-1 * np.pi, np.pi]]

BASE_HEIGHT = 0.089159
UPPER_ARM_LENGTH = 0.39225
FOREARM_LENGTH = 0.42500
SHOULDER_OFFSET = 0.13585
ELBOW_OFFSET = 0.1197
GOAL_ZONE = 0.5
OBSTACLE_SIZE = 0.2


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)
    
#Class declaration for nodes within the tree
class RRT_Node:
    def __init__(self, conf):
        self.conf = conf
        self.parent = None
        self.children = []

    def set_parent(self, parent):
        self.parent = parent
        
    def get_parent(self):
        return self.parent
        
    def get_conf(self):
        return self.conf
    
    def set_conf(self, conf):
        self.conf = conf

    def add_child(self, child):
        self.children.append(child)

#random sampler that generates new potential node in the tree
def sample_conf():
    conf_is_colliding = True
    
    while(conf_is_colliding):
        random_conf = []
        for joint in UR5_JOINT_INDICES:
            random_conf.append(random.uniform(JOINT_RANGES[joint][0], JOINT_RANGES[joint][1]))
        conf_is_colliding = collision_fn(random_conf)
    
    return random_conf
   
#function to find the nearest tree node to the current randomly sampled potential node based off of Euclidean distance
def find_nearest(rand_node, node_list):
    
    q_start_xyz = cspace_to_wspace(node_list[0].get_conf())
    rand_node_xyz = cspace_to_wspace(rand_node.get_conf())
    
    # initialize nearest node to be start node. Updates if node in node_list beats the start node
    nearest_node = None
    min_distance = find_distance(q_start_xyz, rand_node_xyz)
    
    for node in node_list:
        current_node_xyz = cspace_to_wspace(node.get_conf())
        distance = find_distance(current_node_xyz, rand_node_xyz)
        if(distance <= min_distance):
            min_distance = distance
            nearest_node = node

    return nearest_node
    
#returns 3D Euclidean distance between 2 nodes
def find_distance(node1_xyz, node2_xyz):
    return math.sqrt((node1_xyz[0] - node2_xyz[0]) ** 2 + (node1_xyz[1] - node2_xyz[1]) ** 2 + (node1_xyz[2] - node2_xyz[2]) ** 2)
  
#conversion from workspace to robotic configuration space     
def cspace_to_wspace(conf):
    # joints 1 and 2 by default do not correlate with standard unit circle direction convention (at the default camera location, joints 1 and 2 are positive when moving clockwise). These joints are multiplied by -1 to transform them into the xyz coordinate frame predetermined by the spatial environment
    
    x = (UPPER_ARM_LENGTH * math.cos(-1 * conf[1]) + FOREARM_LENGTH * math.cos(-1 * conf[2] + -1 * conf[1])) * math.cos(conf[0]) + (ELBOW_OFFSET - SHOULDER_OFFSET) * math.sin(conf[0])
    
    y = (UPPER_ARM_LENGTH * math.cos(-1 * conf[1]) + FOREARM_LENGTH * math.cos(-1 * conf[2] + -1 * conf[1])) * math.sin(conf[0]) + (SHOULDER_OFFSET - ELBOW_OFFSET) * math.cos(conf[0])
    
    z = BASE_HEIGHT + UPPER_ARM_LENGTH * math.sin(-1 * conf[1]) + FOREARM_LENGTH * math.sin(-1 * conf[2] + -1 * conf[1])
    #print("cspace_to_wspace xyz:")
    #print([x, y, z])
    return [x, y, z]
    
#function that checks if a straight line path can be drawn between the nearest node and the randomly sampled potential node with no collisions
def steer_to(rand_node, nearest_node):
    step_size = 0.05
    is_collision = False
    
    start = cspace_to_wspace(rand_node.get_conf())
    end = cspace_to_wspace(nearest_node.get_conf())
    
    steps = find_distance(start, end) / step_size
    
    delta_x = (end[0] - start[0]) / steps
    delta_y = (end[1] - start[1]) / steps
    delta_z = (end[2] - start[2]) / steps
    
    dummy_point = start
    
    for i in range(1, int(steps) + 1):
        x_col = False
        y_col = False
        z_col = False
        
        dummy_point[0] += delta_x
        dummy_point[1] += delta_y
        dummy_point[2] += delta_z
        
        
        #obstacle collision checking
        if((dummy_point[0] >= (1/4 - OBSTACLE_SIZE) and dummy_point[0] <= (1/4 + OBSTACLE_SIZE)) or (dummy_point[0] >= (2/4 - OBSTACLE_SIZE) and dummy_point[0] <= (2/4 + OBSTACLE_SIZE))):
            x_col = True
        if(dummy_point[1] >= (0 - OBSTACLE_SIZE) and dummy_point[1] <= (0 + OBSTACLE_SIZE)):
            y_col = True
        if((dummy_point[2] >= (1/2 - OBSTACLE_SIZE) and dummy_point[2] <= (1/2 + OBSTACLE_SIZE)) or (dummy_point[2] >= (2/3 - OBSTACLE_SIZE) and dummy_point[2] <= (2/3 + OBSTACLE_SIZE))):
            z_col = True
            
        if((x_col and y_col and z_col) or dummy_point[2] <= 0):
            return False
        
    return True
    
            
#insertion of node into the tree
def insert_node(old_node, new_node):
    old_node.add_child(new_node)
    new_node.set_parent(old_node)
   
#checks if newest added node is within some buffer zone around goal node
def in_goal(node):
     node_xyz = cspace_to_wspace(node.get_conf())
     
     x_bounds = [goal_position[0] - GOAL_ZONE, goal_position[0] + GOAL_ZONE]
     y_bounds = [goal_position[1] - GOAL_ZONE, goal_position[1] + GOAL_ZONE]
     z_bounds = [goal_position[2] - GOAL_ZONE, goal_position[2] + GOAL_ZONE]
     
     if((x_bounds[0] <= node_xyz[0] and x_bounds[1] >= node_xyz[1]) and (y_bounds[0] <= node_xyz[1] and y_bounds[1] >= node_xyz[1]) and (z_bounds[0] <= node_xyz[2] and z_bounds[1] >= node_xyz[2])):
        return True
     else:
        return False

def RRT():

    q_start = RRT_Node(start_conf)
    
    Tree = [q_start]
    
    q_goal = None
    
    while True:
        q_rand_conf = sample_conf()
        
        if(len(Tree) == 1):
            q_nearest = q_start
            q_parent = q_start
            q_children = []
            
        q_rand = RRT_Node(q_rand_conf)
        q_rand.parent = q_parent
        q_rand.children = q_children
        
        
        # Visualization of end effector
        #remove_marker(end_effector)
        #ee_wspace = cspace_to_wspace(q_rand)
        #end_effector = draw_sphere_marker(position = ee_wspace, radius = 0.1, color = [0, 1, 0, 1])
        q_nearest = find_nearest(q_rand, Tree)
        
        if steer_to(q_rand, q_nearest):
            Tree.append(q_rand)
            insert_node(q_nearest, q_rand)
            draw_sphere_marker(position = cspace_to_wspace(q_rand.get_conf()), radius = 0.02, color = [0, 0, 1, 1])
            if in_goal(q_rand):
                q_goal = q_rand
                break
    
    path = []          
    
    q_current = q_goal
    
    while q_current.get_parent():
        path = [q_current.get_conf()] + path
        q_current = q_current.get_parent()
    
    path.append(goal_conf)
    
    #for conf in path:
        #print(conf)
        
    for conf in path:
        print(cspace_to_wspace(conf))
    
    return path


if __name__ == "__main__":

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal intialization
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    start_marker = draw_sphere_marker(position=start_position, radius=0.02, color=[0, 0, 1, 1])
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    
    cspace_to_wspace(start_conf)
    
	# placeholder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())
    # print(collision_fn(start_conf))
	
    path_conf = RRT()

    if path_conf is None:
        # pause here
        input("no collision-free path is found within the time budget, finish?")
    else:
        # execute the path
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
