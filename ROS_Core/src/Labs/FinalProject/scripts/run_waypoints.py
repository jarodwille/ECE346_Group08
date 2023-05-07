#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.path as mpltPath
import pylanelet

import sys
import os
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('racecar_planner'),'scripts','utils'))
from static_obstacle import get_obstacle_vertices

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import rospy
from nav_msgs.msg import Path
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from task2_world.util import RefPath, get_ros_param,  RealtimeBuffer
# from racecar_planner.utils import get_obstacle_vertices
from geometry_msgs.msg import PoseStamped
from RRT import RRT



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
        ### for simulation
        self.pose_sub = rospy.Subscriber('/Simulation/Pose', Odometry, self.odom_callback, queue_size=10)
        # ### for real truck
        # self.pose_sub = rospy.Subscriber('/SLAM/Pose', Odometry, self.odom_callback, queue_size=10)
        
        self.path_pub = rospy.Publisher('Routing/Path', Path, queue_size=10,latch = True)
        
        # ### Subscriber to get the obstacles (new added)
        self.static_obs_sub = rospy.Subscriber(
            '/Obstacles/Static', MarkerArray, self.static_obstacle_callback, queue_size=10)
        
              
    def odom_callback(self, odom_msg):
        
        self.odom_msg = odom_msg
        
    ### new added callback
    def static_obstacle_callback(self, msg):
        '''
        Static obstacle callback function
        '''
        
        self.obs_position = []
        self.obs = msg

        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.obs_position.append([x, y])   
        
           
    # ### new added function 
    # def calllanelet(self, waypoints, start_index, goal_index, positions):
    #     map = self.map

    #     # Define the start and goal positions as Lanelet points
    #     ### would be current position for planning? need to think
    #     start = pylanelet.Point2d(waypoints[start_index][0], waypoints[start_index][1])
    #     goal = pylanelet.Point2d(waypoints[goal_index][0], waypoints[goal_index][1])

    #     # Get the obstacle vertices in the world frame
    #     obs_verticeslist = []
    #     for index in self.obs_list_rollout:  
    #         obs_id, obs_vertices = get_obstacle_vertices(self.obs[index])
    #         obs_verticeslist.append(obs_vertices)
            

    #     # Convert the vertices to the Lanelet map frame
    #     obs_vertices_map = [pylanelet.Point3d(obs_vertex[0], obs_vertex[1], obs_vertex[2]) for obs_vertex in obs_verticeslist]

    #     # Create a Lanelet polygon representing the obstacle
    #     obstacle = pylanelet.Polygon3d(obs_vertices_map)

    #     # Add the obstacle to the Lanelet map
    #     map.add(obstacle)

    #     # Perform path planning using the A* algorithm
    #     path = map.astar(start, goal, [obstacle])

    #     # Extract the planned path as a list of Lanelet points
    #     path_points = [p.centerline[0] for p in path]
    #     # Conver iot as a nested list representing the x,y position
    #     path_points_xy = [[point.x, point.y] for point in path_points]
        
    #     return path_points_xy
    
    
    # Define a function to calculate the angle between two points
    def angle_between_points(self, x1, y1, x2, y2):
        return np.arctan2(y2-y1, x2-x1)
    
    
    def rollout_obs(self, obs_position):
        selected_obs = []
        self.obs_list_rollout = []
        ### check if the obstackes are inside the redius (r=1) circle center at teh current truck position
        x_curr = self.odom_msg.pose.pose.position.x
        y_curr = self.odom_msg.pose.pose.position.y
        
        for i in range(len(obs_position)):
            # Calculate the Euclidean distance between current position and the obstacle position
            distance = np.linalg.norm(np.array(obs_position[i]) - np.array([x_curr, y_curr]))
            
            # Check if the distance is less than 1 meter and if the obstacle is in front of you
            if distance < 1.0:
                selected_obs.append(obs_position[i])
                self.obs_list_rollout.append(i)
                
        return selected_obs
             
                
    ### new added function
    def new_referencepath(self, pathnav):
        Flag: bool
        Flag = False
        posx = []
        posy = []

        iterpath = pathnav.path
        for pose in iterpath.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            posx.append(x)
            posy.append(y)
          
        waypoints = np.column_stack((posx, posy))
        ### get the positions of the filtered obstacles
        positions = np.array(self.rollout_obs(self.obs_position))
        print('selected obstacle positions', positions)
        
        nearby_indices = []
        for i in range(len(waypoints)):
            distances = np.linalg.norm(positions - waypoints[i], axis=1)
            if np.any(distances < 1.0):
                nearby_indices.append(i)
                        
        
        if len(nearby_indices) == 0:
            pass
        else:
            ### changed logic: if reference path go through obstacles, only replace that part waypoints but not all
            start_index = 0 if nearby_indices[0] == 0 else nearby_indices[0] - 1
            goal_index = len(waypoints) - 1 if nearby_indices[-1] == len(waypoints) - 1 else nearby_indices[-1] + 1
                 
            print("Start rrt planning")

            # Set Initial parameters
            size_column = np.full((positions.shape[0], 1), 0.25)
            positions = np.hstack((positions, size_column)) # add size to positions

            rrt = RRT(start=waypoints[start_index], goal=waypoints[goal_index],
                    randArea=[-10, 10], obstacleList=positions)
            new_ref = rrt.Planning()
            
            # new_ref = self.rrt(waypoints, start_index, goal_index, positions, 1000, 0.1)
            print('new_ref', new_ref)
            
            print("original", waypoints)
            print("start index", start_index)
            print("end index", goal_index)
            # Replace the waypoints with the corresponding waypoints from the replanned path
            waypoints = np.delete(waypoints, slice(start_index, goal_index+1), axis=0)
            print("after delete", waypoints)
            waypoints = np.insert(waypoints, start_index, new_ref, axis=0)
            print("after replace, final", waypoints)

            
            Flag = True
            
        
        return waypoints, Flag
        
        
       
    
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
            
            print('start pos', x_start, y_start)
            
            x_goal = self.goal_array[i][0]# x coordinate of the goal
            y_goal = self.goal_array[i][1]# y coordinate of the goal
                        
            
            plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
            plan_response = self.plan_client(plan_request)
            
            ### Newly added code
            replan, replan_flag = self.new_referencepath(plan_response)
            
            path_msg: Path
            
            print('replan', replan)
            print('-------------------------------------')
            
            if replan_flag == False: ### check if this is empty
                print('not replan---------------------------------------------')
                path_msg = plan_response.path
               
            else: ### convert nested list into nav_msg.msg type
                print('in replan----------------------------------------------')
                path_msg = Path()
                for point in replan:
                    pose = PoseStamped()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = 0.0
                    path_msg.poses.append(pose)
                    
            path_msg.header.stamp = rospy.get_rostime()
            path_msg.header.frame_id = 'map'
                
            self.path_pub.publish(path_msg)
                
            
            
            
            
            ### Nnewly added code ends here
            ## can be commented for now for testing  
            
             
            # path_msg: Path
            # path_msg = plan_response.path
            # path_msg.header.stamp = rospy.get_rostime()
            # path_msg.header.frame_id = 'map'
            # self.path_pub.publish(path_msg)
            
            
            ## can be commented for now for testing
            
            dx = x_goal - x_start
            dy = y_goal - y_start
            
            dist = np.sqrt(dx**2 + dy**2)   
            if dist<0.4:
                i +=1  
                rospy.sleep(0.1)
            else:
                rospy.sleep(1.0)
        rospy.loginfo("Finished!")   
                    
              
                
                    

def main():
    '''
    main 

    '''
    rospy.init_node('run_waypoints', anonymous = True)
    
    init = Waypoints()
    init.calculate_waypoints()
   
if __name__=='__main__':
    main()
    

