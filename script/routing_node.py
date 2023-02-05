#!/usr/bin/env python
import rospy
from routing import Routing

def main():
    rospy.init_node('routing_node')
    rospy.loginfo("Start routing node")
    
    map_file = rospy.get_param("~map_file")
    Routing(map_file)
    
    rospy.spin()

if __name__ == '__main__':
    main()