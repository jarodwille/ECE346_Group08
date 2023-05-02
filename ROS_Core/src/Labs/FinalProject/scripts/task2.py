import rospy
import yaml
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from final_project.srv import Schedule, ScheduleRequest, ScheduleResponse

# set up the service client to start session
client_start = rospy.ServiceProxy('/SwiftHaul/Start', Empty)

# set up the service client to get boss schedule
client_schedule = rospy.ServiceProxy('/SwiftHaul/BossSchedule', Schedule)

# call the service to start session
client_start(EmptyRequest())

# call the service to get boss schedule
respond = client_schedule(ScheduleRequest())

# Retrieve Warehouse Information from yaml file
with open('../task2.yaml', 'r') as stream:
    warehouse_info = yaml.safe_load(stream)
    for warehouse in warehouse_info:
        
