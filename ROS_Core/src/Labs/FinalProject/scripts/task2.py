import rospy
import np
import yaml
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from final_project.srv import Schedule, ScheduleRequest, ScheduleResponse, \
    Task, TaskRequest, TaskResponse, Reward, RewardRequest, RewardResponse
from nav_msgs.msg import Odometry
from racecar_routing.srv import Plan, PlanResponse, PlanRequest


class SwiftHaulTasks:
    def __init__(self):

        self.load_warehouse_info()
        self.setup_clients()
        self.setup_subscriber()

        # response of boss schedule
        self.boss_schedule = self.boss_schedule_client(ScheduleRequest())

    def load_warehouse_info(self):

        # Retrieve Warehouse Information from yaml file
        with open('../task2.yaml', 'r') as stream:
            warehouse_info = yaml.safe_load(stream)

        # HACK: No failsafe is implemented for invalid warehouse config
        self.warehouse_location = []
        self.warehouse_probability = []

        for warehouse in warehouse_info.values():
            location_info = warehouse['location']
            location_info.extend(warehouse['dxdy'])
            self.warehouse_location.append(location_info)
            self.warehouse_probability.append(warehouse['probability'])


        self.num_warehouse = len(self.warehouse_location)

    def setup_clients(self):

        # set up the service client to start session
        rospy.wait_for_service('/SwiftHaul/Start')
        self.start_client = rospy.ServiceProxy('/SwiftHaul/Start', Empty)
        self.start_client(EmptyRequest())  # call the service to start session

        # set up the service client to get boss schedule
        rospy.wait_for_service('/SwiftHaul/BossSchedule')
        self.boss_schedule_client = rospy.ServiceProxy(
            '/SwiftHaul/BossSchedule', Schedule)

        # set up service client for side task
        rospy.wait_for_service('/SwiftHaul/SideTask')
        self.side_task_client = rospy.ServiceProxy('/SwiftHaul/SideTask', Task)

        # set up client for boss task
        rospy.wait_for_service('/SwiftHaul/BossTask')
        self.boss_task_client = rospy.ServiceProxy('/SwiftHaul/BossTask', Task)

        # set up client for rewards
        rospy.wait_for_service('/SwiftHaul/GetReward')
        self.reward_client = rospy.ServiceProxy('/SwiftHaul/GetReward', Reward)

        # set up planning client
        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)

    def setup_subscriber(self):
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

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
            
            x_goal = self.warehouse_location[self.boss_schedule_client[1]][0]# x coordinate of the goal
            y_goal = self.warehouse_location[self.boss_schedule_client[1]][1]# y coordinate of the goal
            
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
    #     # If still aims for thecurrent waypoint, keep replanning.
    #     odom_msg = self.odom_msg

    #     for i in range(len(self.goal_array)):

    #         # Start position
    #         x_start = odom_msg.pose.pose.position.x
    #         y_start = odom_msg.pose.pose.position.y

    #         x_goal = self.goal_array[i][0]  # x coordinate of the goal
    #         y_goal = self.goal_array[i][1]  # y coordinate of the goal

    #         plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
    #         plan_response = self.plan_client(plan_request)

            
    #         # # The following script will generate a reference path in [RefPath](scripts/task2_world/util.py#L65) class, which has been used in your Lab1's ILQR planner
    #         # x = []
    #         # y = []
    #         # width_L = []
    #         # width_R = []
    #         # speed_limit = []

    #         # for waypoint in plan_response.path.poses:
    #         #     x.append(waypoint.pose.position.x)
    #         #     y.append(waypoint.pose.position.y)
    #         #     width_L.append(waypoint.pose.orientation.x)
    #         #     width_R.append(waypoint.pose.orientation.y)
    #         #     speed_limit.append(waypoint.pose.orientation.z)

    #         # centerline = np.array([x, y])

    #         # # This is the reference path that we passed to the ILQR planner in Lab1
    #         # ref_path = RefPath(centerline, width_L, width_R,
    #         #                    speed_limit, loop=False)

    #         # need to rethink this current position logic
    #         curr_x = odom_msg.pose.pose.position.x
    #         curr_y = odom_msg.pose.pose.position.y

    #         dist = np.sqrt((curr_x - x_goal)**2 + (curr_y - y_goal)**2)

    #         while dist > 2.0:
    #             rospy.sleep(1.0)


if __name__ == '__main__':

    rospy.init_node('task2.py')

    init = SwiftHaulTasks()
    while not rospy.is_shutdown():
        init.calculate_waypoints()
