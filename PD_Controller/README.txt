This is a PD controller I made for a 2DOF robotic arm.
This uses the reacher robotic arm as part of the pygym library.
The given trajectory is in blue (you may have to zoom in a lot when you run the robot_arm.py script to see it), and the PD trajectory is in red. I tuned the controller to get the error on the order of 10^-5
The PD controller takes in the current XY value and calculates the error between it and the ideal value. It then calculates the Fx and Fy forces needed to shove the end effector to more align with the ideal trajectory. I tried to make this using first principles, so minimal libraries were used.
