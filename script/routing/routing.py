from util import load_lanelet_map, map_to_markerarray, get_ros_param
import lanelet2
import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Routing:
    def __init__(self, map_file):
        self.lanelet_map = load_lanelet_map(map_file)
        
        self.read_parameters()
        self.setup_publisher()
        self.setup_subscriber() 
        
        # Container to store the Route information
        self.cur_pose = None
        self.cur_goal = None
        self.cur_route = None
        
    def read_parameters(self):
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        
    def setup_publisher(self):
        self.map_pub = rospy.Publisher('Map', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('Path', Path, queue_size=10)
        
    def setup_subscriber(self):
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=1)
        # This subscribe to the 2D Nav Goal in RVIZ
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
    
    def goal_callback(self, goal_msg):
        if self.cur_pose is None:
            rospy.warnonce("No pose received yet, cannot compute path.")
        
    
    def odometry_callback(self, odom_msg):
        """
        Subscriber callback function of the robot pose
        """
        pass
        
    def publish_map(self):
        marker_array = map_to_markerarray(self.lanelet_map)
        self.map_pub.publish(marker_array)

    def getRoute(self, start, goal):