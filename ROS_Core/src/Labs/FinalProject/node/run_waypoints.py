#!/usr/bin/env python
import rospy
import numpy as np
import pickle

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Waypoints:
    def __init__(self):
         ## read from parameters
        self.waypoints_order = rospy.get_param("goal_order")
        self.waypoints_array = np.zeros((1, 2))

        for i in range(12):
            goal_name = "goal_" + str(i+1)
            goal_read = rospy.get_param(goal_name)
            print("goal_read: {}, shape: {}".format(goal_read, np.array(goal_read).shape))
            self.waypoints_array = np.concatenate((self.waypoints_array, np.array(goal_read).reshape(1, 2)), axis=0)
        
        self.waypoints_array = np.delete(self.waypoints_array, 0, 0)

        self.goal_array = np.zeros((len(self.waypoints_order), 2))
        
        for i in range(len(self.waypoints_order)):
            index = self.waypoints_order[i] # goal index in goal order list
            self.goal_array[i, :] = self.waypoints_array[index-1, :]
            
        print('goal array', self.goal_array)


        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        ### call service request to routing
           

    def calculate_waypoints(self):
        goal = PoseStamped()
        goal.header.frame_id = 'world_frame'
        goal.pose.position.x = self.goal_array[0][0]
        goal.pose.position.y = self.goal_array[0][1]
        self.goal_pub.publish(goal)



def main():
    '''
    main 

    '''
    rospy.init_node('run_waypoints', anonymous = True)
    
    init = Waypoints()

    while not rospy.is_shutdown():
         init.calculate_waypoints()

   
    rospy.spin()

if __name__=='__main__':
    main()
    

