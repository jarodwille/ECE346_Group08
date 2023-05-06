#!/usr/bin/env python
import rospy
import numpy as np
import yaml
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from final_project.srv import Schedule, ScheduleRequest, ScheduleResponse, \
    Task, TaskRequest, TaskResponse, Reward, RewardRequest, RewardResponse
from nav_msgs.msg import Odometry, Path
from racecar_routing.srv import Plan, PlanResponse, PlanRequest


class SwiftHaulTasks:
    def __init__(self):
        print("STArtED PROGRAM 0")
        self.load_warehouse_info()
        self.setup_clients()
        self.setup_sub_pub()

        # response of boss schedule
        rospy.sleep(15)
          # call the service to start session
        self.boss_schedule = self.boss_schedule_client(ScheduleRequest())
        print("Starting client at", rospy.get_rostime())
        self.start_client(EmptyRequest())
        self.odom_msg = None

    def load_warehouse_info(self):

        # Retrieve Warehouse Information from yaml file
        with open('/home/maddie/ECE346_Group08/ROS_Core/src/Labs/FinalProject/task2.yaml', 'r') as stream:
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

        # set up the service client to get boss schedule (gets full schedule)
        rospy.wait_for_service('/SwiftHaul/BossSchedule')
        self.boss_schedule_client = rospy.ServiceProxy(
            '/SwiftHaul/BossSchedule', Schedule)

        # set up service client for side task (request to complete a side task)
        rospy.wait_for_service('/SwiftHaul/SideTask')
        self.side_task_client = rospy.ServiceProxy('/SwiftHaul/SideTask', Task)

        # set up client for boss task (request to complete a boss task)
        rospy.wait_for_service('/SwiftHaul/BossTask')
        self.boss_task_client = rospy.ServiceProxy('/SwiftHaul/BossTask', Task)

        # set up client for rewards
        rospy.wait_for_service('/SwiftHaul/GetReward')
        self.reward_client = rospy.ServiceProxy('/SwiftHaul/GetReward', Reward)

        # set up planning client
        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)

    def setup_sub_pub(self):
        self.pose_sub = rospy.Subscriber('/Simulation/Pose', Odometry, self.odom_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/Routing/Path', Path, queue_size=10, latch = True)

    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg

    def plan_path(self):
        # If still aims for thecurrent waypoint, keep replanning.
        i = 0
        rospy.sleep(2.0)
        warehouse_idx = None
        
        while not rospy.is_shutdown() and i<self.num_warehouse:
            if self.odom_msg == None:
                "No odom_msg received"
                rospy.sleep(0.1)
                continue
        
            # Start position
            x_start = self.odom_msg.pose.pose.position.x
            y_start = self.odom_msg.pose.pose.position.y

            # warehouse_idx= self.boss_schedule.goal_warehouse_index[i] ##we don't want the boss schudule we only want boss task (only need sechdule for rejoing after side task)
            # request a task from the boss
            try:
                if warehouse_idx is None:
                    warehouse_idx = self.boss_task_client(TaskRequest()).task
                    print("For this Task Go to Warehouse: ", warehouse_idx)
            except Exception as e:
                continue

            # Get information about the goal warehouse
            x_goal = self.warehouse_location[warehouse_idx][0]# x coordinate of the goal
            y_goal = self.warehouse_location[warehouse_idx][1]# y coordinate of the goal
            dx = self.warehouse_location[warehouse_idx][2]# x width of the warehouse
            dy = self.warehouse_location[warehouse_idx][3]# y width of the warehouse
            print("current location", x_start, ", ", y_start)
            print("goal location", x_goal, ", ", y_goal)
            print("dx: ", dx, "dy: ", dy)
            print("how far away x: " ,np.abs(x_start - x_goal), "how far away y: ", np.abs(y_start-y_goal))

            # plan a reference trajectory
            plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
            print("request planned")
            plan_response = self.plan_client(plan_request)
            print("send plan to planning client to get plan response")
            
            path_msg: Path
            path_msg = plan_response.path
            path_msg.header.stamp = rospy.get_rostime()
            path_msg.header.frame_id = 'map'
            self.path_pub.publish(path_msg)
            print("publish path")
           
            # if we arrive at warehouse
            if np.abs(x_start - x_goal) < dx*0.5 and np.abs(y_start - y_goal) < dy*0.5:
                print("our distance from goal: ",np.abs(x_start - x_goal),"< dx: ", dx )

                print("Truck inside warehouse")
                # calculate reward for completing a task
                reward_response = self.reward_client(RewardRequest(warehouse_idx)) # check with boss if we made it
                print("Boss agrees that complete task and gives us the reward")
                if reward_response.done == False:
                    print("boss is unsatisfied with our arrival")
                
                print("Total reward so far: ", reward_response.total_reward)

                
                # stop (plan route to self)! if boss hasn't started new task (meaning boss stopped)
                while(self.boss_task_client(TaskRequest()).task == -1):
                    print("No new task received")
                    #plan_request = PlanRequest([x_start, y_start], [x_start, y_start])
                    #plan_response = self.plan_client(plan_request)
                    #path_msg = plan_response.path
                    #path_msg.header.stamp = rospy.get_rostime()
                    #path_msg.header.frame_id = 'map'
                    #self.path_pub.publish(path_msg)
                    rospy.sleep(0.1)

                i += 1
                rospy.sleep(0.1)
                warehouse_idx = None
            else:
                rospy.sleep(1)
        rospy.loginfo("Finished!")   
        

if __name__ == '__main__':
    print("entered main")
    rospy.init_node('ego_simulation_node')
    print("initialized node")
    init = SwiftHaulTasks()
    print("ran swifthaultasks")
    while not rospy.is_shutdown():
        init.plan_path()
