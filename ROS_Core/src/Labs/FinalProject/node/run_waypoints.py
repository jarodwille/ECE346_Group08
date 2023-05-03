#!/usr/bin/env python
import rospy
import numpy as np

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import rospy
from racecar_routing.srv import Plan, PlanResponse, PlanRequest, PlanClient
from .util import map_to_markerarray, get_ros_param
from Lab1.scripts.ILQR.ref_path import RefPath

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
            
        rospy.wait_for_service('/routing/plan')
        plan_client = rospy.ServiceProxy('/routing/plan', Plan)
            
        ## setup the position subscriber
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        
        
    def read_parameters(self):
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        
        
    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg
        
    
    def calculate_waypoints(self):
        # If still aims for thecurrent waypoint, keep replanning.
        odom_msg = self.odom_msg
        
        
        for i in range(len(self.goal_array)):
            
            # Start position
            x_start = odom_msg.pose.pose.position.x
            y_start = odom_msg.pose.pose.position.y
            
            x_goal = self.goal_array[i][0]# x coordinate of the goal
            y_goal = self.goal_array[i][1]# y coordinate of the goal
            
            plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
            plan_response = plan_client(plan_request)
            
            # The following script will generate a reference path in [RefPath](scripts/task2_world/util.py#L65) class, which has been used in your Lab1's ILQR planner
            x = []
            y = []
            width_L = []
            width_R = []
            speed_limit = []
            
            for waypoint in plan_response.path.poses:
                x.append(waypoint.pose.position.x)
                y.append(waypoint.pose.position.y)
                width_L.append(waypoint.pose.orientation.x)
                width_R.append(waypoint.pose.orientation.y)
                speed_limit.append(waypoint.pose.orientation.z)
                
            centerline = np.array([x, y])
            
            # This is the reference path that we passed to the ILQR planner in Lab1
            ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=False)
            
            ### need to rethink this current position logic
            curr_x = odom_msg.pose.pose.position.x
            curr_y = odom_msg.pose.pose.position.y


            dist = np.sqrt((curr_x - x_goal)**2 + (curr_y - y_goal)**2)
            
            while dist > 2.0:
                rospy.sleep(1.0)           
        
        
           

    # def calculate_waypoints(self):
        
    #     for i in range(len(self.goal_array)):
    #         # Get the request for the next waypoint from service in routing.py
    #         request = call_waypointsRequest()
    #         # Put the waypoint into the PoseStamped() object as cumstomized in call_waypoints.srv in srv folder
    #         goal = PoseStamped()
    #         goal.header.frame_id = 'world_frame'
    #         goal.pose.position.x = self.goal_array[i][0]
    #         goal.pose.position.y = self.goal_array[i][1]
    #         # print('Goal--------------------------------')
    #         # print('goalx: {}, goaly: {}'.format(goal.pose.position.x, goal.pose.position.y))
    #         # print('-------------------------------------')
            
    #         request.Pose = goal
    #         response = self.goal_srv(request)

    #         # If haven't recevied the request to be True, keep calling and until routing.py request, pass to the next goal in for loop
    #         while not response.success:
    #             response = self.goal_srv(request) ### if not success, keep call the service
                    
              
                
                    

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
    

