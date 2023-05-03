#!/usr/bin/env python
import rospy
import numpy as np

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import rospy
from nav_msgs.msg import Path
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from task2_world.util import RefPath, get_ros_param,  RealtimeBuffer
# from Labs.FinalProject.scripts.task2_world.util import get_ros_param

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
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)
        
        self.odom_msg = None
        self.pose_sub = rospy.Subscriber('/Simulation/Pose', Odometry, self.odom_callback, queue_size=10)
        
        self.path_pub = rospy.Publisher('Routing/Path', Path, queue_size=10,latch = True)
        
        
              
    def odom_callback(self, odom_msg):
        
        self.odom_msg = odom_msg
        
    
    def calculate_waypoints(self):
        # If still aims for thecurrent waypoint, keep replanning.
        i = 0
        
        while not rospy.is_shutdown() and i<len(self.goal_array):
            if self.odom_msg == None:
                rospy.sleep(0.1)
                continue
        
            # Start position
            x_start = self.odom_msg.pose.pose.position.x
            y_start = self.odom_msg.pose.pose.position.y
            
            x_goal = self.goal_array[i][0]# x coordinate of the goal
            y_goal = self.goal_array[i][1]# y coordinate of the goal
            
            plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
            plan_response = self.plan_client(plan_request)
                
            path_msg: Path
            path_msg = plan_response.path
            path_msg.header.stamp = rospy.get_rostime()
            path_msg.header.frame_id = 'map'
            self.path_pub.publish(path_msg)
           

            dist = np.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2)   
            if dist<0.4:
                i +=1  
                rospy.sleep(0.1)
            else:
                rospy.sleep(1)
        rospy.loginfo("Finished!")   
        
        
           

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
    init.calculate_waypoints()
   
if __name__=='__main__':
    main()
    

