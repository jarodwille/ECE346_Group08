#!/usr/bin/env python
import rospy
from routing import load_lanelet_map, map_to_markerarray
from visualization_msgs.msg import MarkerArray

def main():
    rospy.init_node('routing_node')
    rospy.loginfo("Start routing node")
    
    map = load_lanelet_map("/hdd/ROS_dir/routing/src/PrincetonRaceCar_routing/script/routing/build_lanelet_map/track.osm")
    marker_array = map_to_markerarray(map)
    
    pub = rospy.Publisher('lanelet_marker_array', MarkerArray, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        pub.publish(marker_array)
        rate.sleep()


if __name__ == '__main__':
    main()