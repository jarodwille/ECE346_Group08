#!/usr/bin/env python
import rospy
import numpy as np
import pickle

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from FinalProject.srv import call_waypoints, call_waypointsRequest

class Waypoints:
    def __init__(self):
         ## read from parameters
        self.waypoints_order = rospy.get_param("goal_order")
        self.waypoints_array = np.zeros((1, 2))

        for i in range(12):
            goal_name = "goal_" + str(i+1)
            goal_read = rospy.get_param(goal_name)
            # print("goal_read: {}, shape: {}".format(goal_read, np.array(goal_read).shape))
            self.waypoints_array = np.concatenate((self.waypoints_array, np.array(goal_read).reshape(1, 2)), axis=0)
        
        self.waypoints_array = np.delete(self.waypoints_array, 0, 0)

        self.goal_array = np.zeros((len(self.waypoints_order), 2))
        
        for i in range(len(self.waypoints_order)):
            index = self.waypoints_order[i] # goal index in goal order list
            self.goal_array[i, :] = self.waypoints_array[index-1, :]
            
        # print('goal array', self.goal_array)
        
        rospy.wait_for_service('call_waypoints')
        
        ### Initialize the service. This atims to get the service request from routing.py
        self.goal_srv = rospy.ServiceProxy('call_waypoints', call_waypoints)
        # self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) 
        
           

    def calculate_waypoints(self):
        
        for i in range(len(self.goal_array)):
            # Get the request for the next waypoint from service in routing.py
            request = call_waypointsRequest()
            # Put the waypoint into the PoseStamped() object as cumstomized in call_waypoints.srv in srv folder
            goal = PoseStamped()
            goal.header.frame_id = 'world_frame'
            goal.pose.position.x = self.goal_array[i][0]
            goal.pose.position.y = self.goal_array[i][1]
            # print('Goal--------------------------------')
            # print('goalx: {}, goaly: {}'.format(goal.pose.position.x, goal.pose.position.y))
            # print('-------------------------------------')
            
            request.Pose = goal
            response = self.goal_srv(request)

            # If haven't recevied the request to be True, keep calling and until routing.py request, pass to the next goal in for loop
            while not response.success:
                response = self.goal_srv(request) ### if not success, keep call the service
                    
              
                
                    

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
    

