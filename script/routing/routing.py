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

from dynamic_reconfigure.server import Server
from racecar_routing.cfg import routingConfig
from racecar_routing.srv import Plan, PlanResponse


class Routing:
    def __init__(self, map_file):
        self.read_parameters()

        with open(map_file, 'rb') as f:
            self.lanelet_map = pickle.load(f)

        self.goal_with_heading = False

        self.lanelet_map.build_graph(self.lane_change_cost)

        self.dyn_server = Server(routingConfig, self.reconfigure_callback)

        self.setup_sub_pub()
        self.setup_service()

    def reconfigure_callback(self, config, level):
        self.goal_with_heading = config['goal_with_heading']
        rospy.loginfo(f"Set goal_with_heading to {self.goal_with_heading}")
        return config

    def read_parameters(self):
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        self.lane_change_cost = get_ros_param('~lane_change_cost', 0.5)
        self.click_goal = get_ros_param('~click_goal', True)

    def setup_sub_pub(self):
        if self.click_goal:  # This is for the case when we click on the map to set the goal
            self.path_pub = rospy.Publisher(
                'Routing/Path', Path, queue_size=10)
            self.odom_msg = None
            self.pose_sub = rospy.Subscriber(
                self.odom_topic, Odometry, self.odom_callback, queue_size=1)
            # This subscribes to our own published goal
            self.goal_pub = rospy.Publisher(
                '/routing/goal', PoseStamped, queue_size=1)
            self.goal_msg = PoseStamped()
            self.goal_msg.pose.position.x, self.goal_msg.pose.position.y = 3.15, 0.15

            self.goal_pub.publish(self.goal_msg)
            self.goal_sub = rospy.Subscriber(
                '/routing/goal', PoseStamped, self.replan_callback, queue_size=1)

    def setup_service(self):
        self.plan_srv = rospy.Service('/routing/plan', Plan, self.plan_srv_cb)

    def plan_srv_cb(self, req):
        response = PlanResponse()
        response.path = self.plan_route(req.start, req.goal)
        return response

    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg

    def replan_callback(self, goal_msg):
        if self.odom_msg is None:
            return
        odom_msg = self.odom_msg

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

        rospy.loginfo(
            f"Planning route from [{start_x:.2f}, {start_y:.2f}, {start_psi:.2f}] to [{goal_x:.2f}, {goal_y:.2f}, {goal_psi:.2f}]")

        planned_path = self.plan_route(
            start_pose, goal_pose, True, self.goal_with_heading, goal_msg.header)
        if planned_path is not None:
            self.path_pub.publish(planned_path)

    def plan_route(self, start_pose, goal_pose, start_with_heading=False, goal_with_heading=False, header=None):
        ''' 
        This is the main function to plan the route
        '''

        path = self.lanelet_map.get_shortest_path(
            start_pose, goal_pose, start_with_heading, goal_with_heading)

        if path is None:
            rospy.logwarn('No path found')
            return None

        path_msg = Path()
        if header is not None:
            path_msg.header = header
        for waypoint in path:
            temp = PoseStamped()
            temp.header = temp.header
            temp.pose.position.x = waypoint[0]
            temp.pose.position.y = waypoint[1]
            temp.pose.orientation.x = waypoint[2]  # left width
            temp.pose.orientation.y = waypoint[3]  # right width
            temp.pose.orientation.z = waypoint[4]  # speed limit
            path_msg.poses.append(temp)

        return path_msg
