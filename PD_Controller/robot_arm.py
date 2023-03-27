import gym
import numpy as np
import pybulletgym.envs
import matplotlib.pyplot as plt
import pybullet
import math
import time

l0 = 0.1
l1 = 0.11

start = [np.pi, 0]

squared_error = []

time_array = []

#generating ideal trajectory
x = [(0.19 + 0.02 * np.cos(theta * 4)) * np.cos(theta) for theta in
np.arange(-np.pi, np.pi, 0.001)]
y = [(0.19 + 0.02 * np.cos(theta * 4)) * np.sin(theta) for theta in
np.arange(-np.pi, np.pi, 0.001)]

#arrays for storing controller position values
x_control = []
y_control = []

x_ik = []
y_ik = []


#create ideal trajectory plot
plt.figure(1)
plt.plot(x, y, 'b')
traj = list(zip(x,y))

def getForwardModel(q0, q1):
    x = l0 * np.cos(q0) + l1 * np.cos(q0 + q1)
    y = l0 * np.sin(q0) + l1 * np.sin(q0 + q1)
    
    return [x, y]
    
def getJacobian(q0, q1):
    dxdq0 = -1 * l0 * np.sin(q0) - l1 * np.sin(q0 + q1)
    dxdq1 = -1 * l1 * np.sin(q0 + q1)
    dydq0 = l0 * np.cos(q0) + l1 * np.cos(q0 + q1)
    dydq1 = l1 * np.cos(q0 + q1)
    
    return np.array([[dxdq0, dxdq1],[dydq0, dydq1]])
    

def xyController(index, q0, q1):
    global prev_position, current_position, prev_e, x_control, y_control

    current_position = getForwardModel(q0, q1)
    
    prev_x = prev_position[0]
    prev_y = prev_position[1]
    
    current_x = current_position[0]
    current_y = current_position[1]
    
    ex = traj[index][0] - current_x
    ey = traj[index][1] - current_y
    
    squared_error.append(math.sqrt(ex ** 2 + ey ** 2))
    
    delta_t = current_time - prev_time
    
    edotx = (ex - prev_e[0]) / delta_t
    edoty = (ey - prev_e[1]) / delta_t
    
    kp = 3500
    kd = 0.1
    
    Fx = kp * ex + kd * edotx
    Fy = kp * ey + kd * edoty
    
    F_vect = np.array([[Fx], [Fy]])
    
    J = getJacobian(q0, q1)
    
    Jt = np.transpose(J)
    
    T_vect = np.transpose(np.matmul(Jt, F_vect))[0]
    
    prev_e = [ex, ey]
    prev_position = current_position
    
    x_control.append(current_position[0])
    y_control.append(current_position[1])
    
    return T_vect


env = gym.make("ReacherPyBulletEnv-v0", new_step_api = True)

#rendering if visualization is wanted
#env.render(mode="human")

env.reset()

#initialization to start position of end effector
env.unwrapped.robot.central_joint.reset_position(start[0], 0)
env.unwrapped.robot.elbow_joint.reset_position(start[1], 0)

#initializing previous and current values
prev_position = getForwardModel(start[0], start[1])
current_position = prev_position

#initializing start action
action = [0, 0]

#initializing time variables for edot calculations
prev_time = time.time()
current_time = time.time()

#setting random low error values for initial 'dummy previous' error (used for edot calculations)
prev_e = [0.1, 0.1]

pybullet.resetDebugVisualizerCamera(1, 5, -80, np.array([0,0,0]))

for i in range(len(traj)):
    
    obs, reward, terminated, truncated, info = env.step(action)
    
    current_time = time.time()
    time_array.append(current_time)
    
    if(terminated):
        obs = env.reset()

    q0, q0_dot = env.unwrapped.robot.central_joint.current_position()
    q1, q1_dot = env.unwrapped.robot.elbow_joint.current_position()
    
    current_state = [q0, q1]
    
    action = xyController(i, q0, q1)
    
    prev_time = current_time
    

  
#plotting final controller trajectory
plt.plot(x_control, y_control, 'r')
plt.axis('equal')
plt.legend(['Ideal Trajectory', 'PD Trajectory'])
plt.title("Ideal Trajectory vs. PD Trajectory of 2DOF Robot Arm")
plt.show()
 
print("Mean squared error for is {}".format(np.mean(squared_error)))
plt.figure(2)
plt.plot(time_array, squared_error)
plt.title("mean squared error over time")
plt.show()




