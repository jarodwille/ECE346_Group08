import rospy
import numpy as np
import pickle

# ROS related imports
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from .util import map_to_markerarray, get_ros_param
import message_filters
from dynamic_reconfigure.server import Server
from racecar_routing.cfg import routingConfig

class Routing:
    def __init__(self, map_file):
        self.read_parameters()
        
        with open(map_file, 'rb') as f:
            self.lanelet_map = pickle.load(f)
            
        self.goal_with_heading = False
        
        self.lanelet_map.build_graph(self.lane_change_cost)
        
        self.dyn_server = Server(routingConfig, self.reconfigure_callback)
        
        self.setup_publisher()
        self.setup_subscriber() 
                    
    def reconfigure_callback(self, config, level):
        self.goal_with_heading = config['goal_with_heading']
        rospy.loginfo(f"Set goal_with_heading to {self.goal_with_heading}")
        return config
        
    def read_parameters(self):
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        self.lane_change_cost = get_ros_param('~lane_change_cost', 0.5)
        
    def setup_publisher(self):
        self.path_pub = rospy.Publisher('Routing/Path', Path, queue_size=10)
        
    def setup_subscriber(self):
        pose_sub = message_filters.Subscriber(self.odom_topic, Odometry)
        # This subscribe to the 2D Nav Goal in RVIZ
        goal_sub = message_filters.Subscriber('/move_base_simple/goal', PoseStamped)
        self.replan_callback = message_filters.ApproximateTimeSynchronizer([pose_sub, goal_sub], 10, 0.1)
        self.replan_callback.registerCallback(self.replan)
        
    def replan(self, odom_msg, goal_msg):
        start_x = odom_msg.pose.pose.position.x
        start_y = odom_msg.pose.pose.position.y
        
        q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, 
                odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        start_psi = euler_from_quaternion(q)[-1]
        start_pose = np.array([start_x, start_y, start_psi])
        
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y
        q = [goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, 
                goal_msg.pose.orientation.z, goal_msg.pose.orientation.w]
        goal_psi = euler_from_quaternion(q)[-1]
        goal_pose = np.array([goal_x, goal_y, goal_psi])
                
        rospy.loginfo(f"Planning route from [{start_x:.2f}, {start_y:.2f}, {start_psi:.2f}] to [{goal_x:.2f}, {goal_y:.2f}, {goal_psi:.2f}]")
        path = self.lanelet_map.get_shortest_path(start_pose, goal_pose, True, self.goal_with_heading)
        
        if path is None:
            rospy.logwarn('No path found')
            return
        
        path_msg = Path()
        path_msg.header = odom_msg.header
        for waypoint in path:
            temp = PoseStamped()
            temp.header = temp.header
            temp.pose.position.x = waypoint[0]
            temp.pose.position.y = waypoint[1]
            temp.pose.orientation.x = waypoint[2] # left width
            temp.pose.orientation.y = waypoint[3] # right width
            temp.pose.orientation.z = waypoint[4] # speed limit
            path_msg.poses.append(temp)
            
        
        self.path_pub.publish(path_msg)
        
