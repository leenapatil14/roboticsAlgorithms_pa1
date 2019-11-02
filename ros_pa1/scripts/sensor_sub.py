#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import *
import tf

def pub_Twist(input_linear_x,input_linear_y,input_linear_z,input_angular_x,input_angular_y,input_angular_z):
   vel_m=Twist()
   vel_m.linear.x=input_linear_x
   vel_m.linear.y=input_linear_y
   vel_m.linear.z=input_linear_z
   vel_m.angular.x=input_angular_x
   vel_m.angular.y=input_angular_y
   vel_m.angular.z=input_angular_z
   publisher_demo=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
   publisher_demo.publish(vel_m)

def stop_agent():
   stop_l_x=0.0
   stop_l_y=0.0
   stop_l_z=0.0
   stop_a_x=0.0
   stop_a_y=0.0
   stop_a_z=0.0
   pub_Twist(stop_l_x,stop_l_y,stop_l_z,stop_a_x,stop_a_y,stop_a_z)

#Goal-seek
def move_forward_agent():
   global distance
   global agent_state
   global prev_state
   prev_state=agent_state
   agent_state="GOALSEEK"
   if(distance<1):
      move_l_x=0.0
   else:  
      move_l_x=0.3
   move_l_y=0.0
   move_l_z=0.0
   move_a_x=0.0
   move_a_y=0.0
   move_a_z=0.0
   pub_Twist(move_l_x,move_l_y,move_l_z,move_a_x,move_a_y,move_a_z)

#detect obstacle
def detect_obstacle(input_range,input_divisions):
   get_regions=[]
   for i in input_divisions:
      if(input_range[i] < 1):
         if(0 <= i <72):
            get_regions.append("R")
         elif(72 <= i < 144):
            get_regions.append("FR")
         elif(144 <= i < 216):
            get_regions.append("F")
         elif(216 <= i < 288):
            get_regions.append("FL")
         elif(288 <= i <= 360):
            get_regions.append("L")
   get_regions = list(set(get_regions))
   return get_regions      

#align agent towards the goal_position
def rotate_agent_to_goal(start_p,goal_p):
   global distance
   global isStart
   global hitpoint
   print("rotating")
   initial_quaternion=(
      start_p.orientation.x,
      start_p.orientation.y,
      start_p.orientation.z,
      start_p.orientation.w
   )
   input_euler=tf.transformations.euler_from_quaternion(initial_quaternion)
   ang=fabs(mline_inclination-input_euler[2])
   msg_twist=Twist()  
   if(ang>0.1):
      msg_twist.angular.z=-0.3
      msg_twist.linear.x=0.0
   else:
      msg_twist.linear.x=0.0
      msg_twist.angular.z=0.0
      print("stop rotating")
      isStart=False
   publisher_demo=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
   publisher_demo.publish(msg_twist)
   if(hitpoint):
      hitpoint=False

#At first callback, rotate the agent towards goal
def get_line_from_start():
   print("start")
   global slope
   global constant_of_line
   global mline_inclination
   global goal_position
   global current_position
   global obs_data
   global hitpoint
   global isStart
   global start_position
   start_position=current_position
   y_diff=(goal_position.position.y-start_position.position.y)
   x_diff=(goal_position.position.x-start_position.position.x)
   if(x_diff!=0):
      slope=y_diff/x_diff
      constant_of_line=goal_position.position.y-(slope*goal_position.position.x)
      mline_inclination=atan2(y_diff,x_diff)
      rotate_agent_to_goal(start_position,goal_position)   

#get perpendicular distance from current point to the m-line(usedto change states)
def get_line_from_current():
   global goal_position
   global current_position
   global start_position
   global mline_pt
   global distance
   y_diff=(goal_position.position.y-start_position.position.y)
   x_diff=(goal_position.position.x-start_position.position.x)
   num = fabs(y_diff * current_position.position.x - (x_diff) * current_position.position.y + (goal_position.position.x * start_position.position.y) - (goal_position.position.y * start_position.position.x))
   den = sqrt(pow(goal_position.position.y - start_position.position.y, 2) + pow(goal_position.position.x - start_position.position.x, 2))
   distance=sqrt((goal_position.position.x-current_position.position.x)**2+(goal_position.position.y-current_position.position.y)**2)
   mline_pt=num / den   

#Wall-follow
def wallfollow():
      global agent_state
      global prev_state
      stop_agent()
      prev_state=agent_state
      agent_state="WALLFOLLOW"
      print("wallfollow")
      msg_twist=Twist()
      if((('F' in obs_data) & (len(obs_data)==1)) | (('L' in obs_data) & ('FR' in obs_data) & (len(obs_data)==2)) | (('FL' in obs_data) & ('R' in obs_data) & (len(obs_data)==2))):
         msg_twist.linear.x=0.1
         msg_twist.angular.z=0.0
      elif(('F' in obs_data) & ('FR' in obs_data) & ('FL' in obs_data) & (len(obs_data)==3)):   
         msg_twist.linear.x=0.0
         msg_twist.angular.z=-0.3           
      elif((('L' in obs_data) & ('FL' in obs_data) & (len(obs_data)==2)) | (('F' in obs_data) & ('FL' in obs_data) & (len(obs_data)==2))):
         msg_twist.linear.x=0.1
         msg_twist.angular.z=0.0
      elif((('F' in obs_data) & ('FR' in obs_data) & (len(obs_data)==2)) | (('R' in obs_data) & ('FR' in obs_data) & (len(obs_data)==2)) | (('FR' in obs_data) & (len(obs_data)==1))):
         msg_twist.linear.x=0.0
         msg_twist.angular.z=-0.3
      elif((('F' in obs_data) & ('FL' in obs_data) &('L' in obs_data) & (len(obs_data)==3)) | (('L' in obs_data) & ('FL' in obs_data) &('FR' in obs_data) & (len(obs_data)==3)) | (('FL' in obs_data) & ('FR' in obs_data) &('R' in obs_data) & (len(obs_data)==3)) | (('R' in obs_data) & ('F' in obs_data) &('FL' in obs_data) & (len(obs_data)==3)) | (('FR' in obs_data) & ('FL' in obs_data) &('L' in obs_data) & (len(obs_data)==3))):
         msg_twist.linear.x=0.0
         msg_twist.angular.z=-0.3 
      elif(('L' in obs_data) & (len(obs_data)==1)):
         msg_twist.linear.x=0.1
         msg_twist.angular.z=0.2  
      elif(('FL' in obs_data) & (len(obs_data)==1)):
         msg_twist.linear.x=0.1
         msg_twist.angular.z=-0.2
      elif((len(obs_data)==5) | (len(obs_data)==4)):
         print("stop")
         msg_twist.linear.x=0.0
         msg_twist.angular.z=-0.2    
      publisher_demo=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
      publisher_demo.publish(msg_twist)

#Get laser-scan data
def callback(data):
   max_range = data.range_max
   global goal_position
   global start_position
   global agent_state
   global prev_state
   global current_position
   global obs_data
   global mline_pt
   global hitpoint
   global distance
   global isStart
   #if first call back orient agent else detect obstacle
   if(not isStart):
      range_array=data.ranges
      int_range=range(len(range_array))
      #get list of regions where obstacle is present
      obs_data=detect_obstacle(range_array,int_range)
      #get direct and perpendicular distance from mline to current position
      get_line_from_current()  
      print(mline_pt,hitpoint,agent_state,prev_state)
      #obstacle
      if((len(obs_data)>0)):
         #agent hit mline for an obstacle first time--Goalseek to wall follow
         if((mline_pt<0.25) & (agent_state=="GOALSEEK") & (prev_state=="GOALSEEK")):
            hitpoint=True
            wallfollow()
         elif((mline_pt<0.25) & (agent_state=="WALLFOLLOW") & (prev_state=="WALLFOLLOW")):
            print("near mline")
            if(hitpoint):
               print("2nd mpoint")
               hitpoint=False
               rotate_agent_to_goal(current_position,goal_position)
            else:
               wallfollow()
         else:
            wallfollow()
      else:
         #no obstacle-goalseek
         move_forward_agent()
   else:
      #orient the agent at start towards mline
      get_line_from_start()
         
#get goal position
def callback_from_homing_beacon(data):
   global goal_position
   goal_position = data.pose

#get current position
def callback_base_pose_ground_truth(data):
   global current_position
   current_position=data.pose.pose



goal_position=PoseStamped().pose
current_position=PoseStamped().pose
start_position=PoseStamped().pose
constant_of_line=0.0
slope=0.0
obs_data=[]
mline_inclination=0.0
agent_state=''
default_direction="LEFT"
mline_pt=0
distance=0
hitpoint=False
isStart=True  
prev_state=''          
def listener():

   rospy.init_node('sensor_sub', anonymous=False)

   rospy.Subscriber('base_pose_ground_truth',Odometry, callback_base_pose_ground_truth)
   rospy.Subscriber('homing_signal', PoseStamped, callback_from_homing_beacon)
   
   rospy.Subscriber('base_scan', LaserScan, callback)
   
    # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
    listener()
