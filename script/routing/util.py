import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .pylanelet import LineType, PyLineString

'''
Utility functions for lanelet2 visualization
'''

def linestring_to_marker(line_string: PyLineString):
    # Helper functions
    def yellow_solid(marker):
        marker.type = Marker.LINE_STRIP
        
        marker.scale.x = 0.02
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 237.0/255.0
        marker.color.g = 212.0/255.0
        marker.color.b = 0.0/255.0
        marker.color.a = 1.0
        
    def white_dashed(marker):
        marker.type = Marker.LINE_LIST
        
        marker.scale.x = 0.01
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 0.5
        
    def white_solid(marker):
        marker.type = Marker.LINE_STRIP
        
        marker.scale.x = 0.01
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 1.0

    def virtual(marker):
        marker.type = Marker.POINTS
        
        marker.scale.x = 0.02
        marker.scale.y = 0.02      
        marker.scale.z = 0
        
        marker.color.r = 192.0/255.0
        marker.color.g = 192.0/255.0
        marker.color.b = 192.0/255.0
        marker.color.a = 0.4
    
    # Main function
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lanelet"
    marker.id = line_string.id #
    marker.action = 0 # add or modify
    marker.lifetime = rospy.Duration(0)
    
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
        
    if line_string.type == LineType.VIRTUAL_LINE:
        virtual(marker)
    elif line_string.type == LineType.WHITE_SOLID:
        white_solid(marker)
    elif line_string.type == LineType.WHITE_DASHED:
        white_dashed(marker)
    elif line_string.type == LineType.DOUBLE_YELLOW:
        yellow_solid(marker)
    else:
        virtual(marker)
    

    num_pt = max(int(line_string.length/0.2)*2, 2)
    sampled_points = line_string.sample_points(0, 1, True, num_pt)
    
    for pt in sampled_points:
        marker.points.append(Point(pt[0], pt[1], 0))
    
    return marker

def map_to_markerarray(lanelet_map):
    marker_array = MarkerArray()
    plotted_linestring = []
    for lanelet in lanelet_map.lanelets.values():
        if lanelet.left_boundary.id not in plotted_linestring:
            marker_array.markers.append(linestring_to_marker(lanelet.left_boundary))
            plotted_linestring.append(lanelet.left_boundary.id)
        if lanelet.right_boundary.id not in plotted_linestring:
            marker_array.markers.append(linestring_to_marker(lanelet.right_boundary))
            plotted_linestring.append(lanelet.right_boundary.id)
            
    return marker_array
    
'''
Utility functions for ROS
'''

def get_ros_param(param_name, default):
    '''
    Read a parameter from the ROS parameter server. If the parameter does not exist, return the default value.
    '''
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    else:
        # try seach parameter
        if param_name[0] == '~':
            search_param_name = rospy.search_param(param_name[1:])
        else:
            search_param_name = rospy.search_param(param_name)

        if search_param_name is not None:
            rospy.loginfo('Parameter %s not found, search found %s, using it', param_name, search_param_name)
            return rospy.get_param(search_param_name)
        else:
            rospy.logwarn("Parameter '%s' not found, using default: %s", param_name, default)
            return default
        
