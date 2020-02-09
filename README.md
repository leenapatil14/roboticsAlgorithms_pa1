# CSE 568 Robotics Algorithm 
## Language-Python
### Masters-SEM1

#### Implementation of BUG-2 Algorithm for robot navigation with obstacle avoidance using ROS.
#### Objective Given:
A 2 wheeled robot is equipped with a 2D laser range finder in order to detect obstacles in its environment. It needs to get to its charging station using a homing signal. The objective of this assignment is to perform perception using a laser range finder, and use the perceived information to avoid obstacles and navigate to the charging station.

#### Tasks performed:
1. Created a ROS node named-'sensor_sub.py' to fetch the data received from the laser sensor. 
2. Used the data to determine obstacles in front of the robot.
3. If Obstacles are found, implemented BUG-2 Algorithm to redirect the robot's pose.
4. Made the robot reach its desired goal position.


##### Issues:
Robot keeps on moving along the obstacle. unable to make it leave the loop.
