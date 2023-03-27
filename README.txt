Here is a compilation of a few code examples I thought would be relevant:

1. catkin_ws is a simple ROS (Noetic) workspace I created for my mechatronics class project. We are building a robot that can collect cubes of different colors on a field. At the start of the game, a certain color is deemed "construction material." The robot is to take all collected construction materials and stack them in a predefined construction zone. All other colored cubes are to be dumnped into the home base zone. This ROS workspace has a script titled publisher.py that reads the image stream from a webcam and classifies the color of a cube presented in front of the webcam for future sorting.
2. Digital_Safe_FSM is a finite state machine that uses a rotary encoder interfaced with Arduino to act as a combo lock for a virtual safe.
3. PD_Controller is a python-based PD controller I made for a 2DOF 2D robotic arm.
4. RRT_Planner is a python-based RRT planning algorithm for a 3DOF 3D robotic arm that traverses from a start node to a goal node while avoiding obstacles along the way.
