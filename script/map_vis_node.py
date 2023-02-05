#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray

import pickle
from routing import map_to_markerarray

def main():
    rospy.init_node('map_visualization_node')
    rospy.loginfo("Start map visualization node")
    
    map_file = rospy.get_param("~map_file")
    
    with open(map_file, 'rb') as f:
        lanelet_map = pickle.load(f)
    
    map_pub = rospy.Publisher('Routing/Map', MarkerArray, queue_size=10)
    
    map_message = map_to_markerarray(lanelet_map)
    
    while not rospy.is_shutdown():
        map_pub.publish(map_message)
        rospy.sleep(0.1)
    

if __name__ == '__main__':
    main()