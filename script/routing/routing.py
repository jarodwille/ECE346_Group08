import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from .lanelet_wrapper import LaneletWrapper
from .util import map_to_markerarray, get_ros_param
import message_filters


class Routing:
    def __init__(self, map_file):
        self.lanelet_wrapper = LaneletWrapper(map_file)
        
        self.read_parameters()
        self.setup_publisher()
        self.setup_subscriber() 
        
        while True:
            self.publish_map()
            if self.map_pub.get_num_connections() > 0:
                break
        
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish_map()
            rate.sleep()
        
    def read_parameters(self):
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        
    def setup_publisher(self):
        self.map_pub = rospy.Publisher('Routing/Map', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('Routing/Path', Path, queue_size=10)
        
    def setup_subscriber(self):
        pose_sub = message_filters.Subscriber(self.odom_topic, Odometry)
        # This subscribe to the 2D Nav Goal in RVIZ
        goal_sub = message_filters.Subscriber('/move_base_simple/goal', PoseStamped)
        self.replan_callback = message_filters.ApproximateTimeSynchronizer([pose_sub, goal_sub], 10, 0.1)
        self.replan_callback.registerCallback(self.replan)
        
    def replan(self, pose, goal):
        pose_x = pose.pose.pose.position.x
        pose_y = pose.pose.pose.position.y
        
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        
        path = self.lanelet_wrapper.get_shortest_path(pose_x, pose_y, goal_x, goal_y)
        if len(path) == 0:
            rospy.logwarn('No path found')
            return
        
        path_msg = Path()
        path_msg.header = pose.header
        for waypoint in path:
            pose = PoseStamped()
            pose.header = pose.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.x = waypoint[2] # left width
            pose.pose.orientation.y = waypoint[3] # right width
            pose.pose.orientation.z = waypoint[4] # speed limit
            path_msg.poses.append(pose)
            
        
        self.path_pub.publish(path_msg)
        
    def publish_map(self):
        marker_array = map_to_markerarray(self.lanelet_wrapper.lanelet_map)
        self.map_pub.publish(marker_array)

