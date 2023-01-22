import lanelet2
from lanelet2.projection import LocalCartesianProjector
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

'''
Utility functions for lanelet2 visualization
'''

def LineString3d_to_marker(line_string, id):
    # Helper functions
    def yellow_solid(marker):
        marker.type = Marker.LINE_STRIP
        
        marker.scale.x = 0.2
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 237.0/255.0
        marker.color.g = 212.0/255.0
        marker.color.b = 0.0/255.0
        marker.color.a = 1.0
        
    def white_dashed(marker):
        marker.type = Marker.LINE_STRIP
        
        marker.scale.x = 0.1
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 1.0
        
    def white_solid(marker):
        marker.type = Marker.LINE_LIST
        
        marker.scale.x = 0.1
        marker.scale.y = 0
        marker.scale.z = 0
        
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 1.0

    def virtual(marker):
        marker.type = Marker.POINTS
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2      
        marker.scale.z = 0
        
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 0.3
    
    # Main function
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lanelet"
    marker.id = id #
    marker.action = 0 # add or modify
    marker.lifetime = rospy.Duration(0)
    
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    
    num_pt = len(line_string)
    
    if line_string.attributes["subtype"] == "solid_solid":
        yellow_solid(marker)
    elif line_string.attributes["subtype"] == "dashed":
        white_dashed(marker)
        num_pt = int(num_pt/2)*2 # make sure the number of points is even
    elif line_string.attributes["subtype"] == "solid":
        white_solid(marker)
    else:
        virtual(marker)

    for i in range(num_pt):
        point = line_string[i]
        marker.points.append(Point(point.x, point.y, 0))
    return marker

def map_to_markerarray(lanelet_map):
    i = 0
    marker_array = MarkerArray()
    linestring_layer = lanelet_map.lineStringLayer
    for line_string in linestring_layer:
        if line_string.attributes["type"] == "line_thin" and "subtype" in line_string.attributes:
            marker = LineString3d_to_marker(line_string, i)
            i+=1
            marker_array.markers.append(marker)
            
    return marker_array
    
def load_lanelet_map(filename):
    '''
    Load a Lanelet2::LaneletMap object from a file
    '''
    projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
    return lanelet2.io.load(filename, projector)


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
        
