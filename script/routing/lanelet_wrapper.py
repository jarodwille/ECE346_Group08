import lanelet2
from lanelet2.core import BasicPoint2d, BasicPoint3d
from lanelet2.projection import LocalCartesianProjector
from lanelet2.geometry import to2D, toArcCoordinates, fromArcCoordinates, ArcCoordinates

from script.routing.util import get_ros_param
import rospy
import numpy as np

'''
This class is a wrapper for lanelet2 objects
It uses the lanelet2 python API to provide a more convenient interface with gerneic data types
'''
class LaneletWrapper:
    def __init__(self, osm_file) -> None:
        self.read_parameters()
        
        self.projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
        
        
        # load lanelet map
        self.lanelet_map = self.load_lanelet_map(osm_file)
        self.lanelet_layer = self.lanelet_map.laneletLayer
        # create routing graph
        
        self.traffic_rules = lanelet2.traffic_rules.create(
                                lanelet2.traffic_rules.Locations.Germany, 
                                lanelet2.traffic_rules.Participants.Vehicle
                            )
        
        routing_cost = [
                lanelet2.routing.RoutingCostDistance(self.lane_change_cost),
                lanelet2.routing.RoutingCostTravelTime(self.lane_change_cost)
            ]
        
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules, routing_cost)
        
        self.warmup()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        
        self.lane_change_cost = get_ros_param("~lane_change_cost", 0.0)
        
    def find_lanelet_by_xy(self, pt):
        '''
        Given:
            x: x coordinate 
            y: y coordinate 
        return:
            dis: distance to the nearest lanelet. 0 if the point is inside the lanelet
            lanelet: the nearest lanelet
        '''
        return lanelet2.geometry.findNearest(self.lanelet_layer, pt, 1)[0]
    
    def get_shortest_path(self, x_start, y_start, x_end, y_end, allow_lane_change = True):
        '''
        Given:
            start_lanelet: the start lanelet
            end_lanelet: the end lanelet
        return:
            path: a list of lanelet ids
        '''
        start_point = BasicPoint2d(float(x_start),float(y_start))
        end_point = BasicPoint2d(float(x_end),float(y_end))
        
        # get the nearest lanelet to the start and end point
        dis_to_start, start_lanelet = self.find_lanelet_by_xy(start_point)
        dis_to_end, end_lanelet = self.find_lanelet_by_xy(end_point)
        
        if dis_to_start > 0:
            rospy.logwarn(f"Start point [{x_start}, {y_start}] is not inside a lanelet")
        
        if dis_to_end > 0:
            rospy.logwarn(f"End point [{x_end}, {y_end}] is not inside a lanelet")
        
        path = self.routing_graph.shortestPath(start_lanelet, end_lanelet, 0, allow_lane_change)
        return self.get_path_centerline(path, start_point, end_point)

    def get_path_centerline(self, path, start_point = None, end_point = None, allow_lane_change = True):
        '''
        Given:
            path: a list of lanelet ids or lanelet.routing.path oubject
        return:
            centerline: a list of centerline points
        '''
        path_centerline = []
        num_lanelet = len(path)
        
        prev_start = 0
        if start_point is not None:
            # add start point
            cl = to2D(path[0].centerline)
            cl_length = lanelet2.geometry.length(cl)
            dis_start = self.get_dis_from_point(cl, start_point.x, start_point.y)
            prev_start = dis_start / cl_length
        
        last_lanelet = path[-1]
        if end_point is not None:
            cl = to2D(last_lanelet.centerline)
            dis_end = self.get_dis_from_point(cl, end_point.x, end_point.y)
        else:
            dis_end = 1
            
        for i in range(num_lanelet-1):
            cur_lanelet = path[i]
            
            next_lanelet = path[i+1]
            relation = self.routing_graph.routingRelation(cur_lanelet, next_lanelet, True)
            if relation == lanelet2.routing.RelationType.Left \
                    or relation == lanelet2.routing.RelationType.Right:
                if i == num_lanelet-2: # need a lane change to the last lanelet
                    cur_end = (dis_end-prev_start)/2 + prev_start
                else:
                    cur_end = (1 - prev_start)/2 + prev_start
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, 
                                                                cur_end, include_end_point = False,
                                                                allow_lane_change = allow_lane_change))
                prev_start = cur_end
            else:
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, 1,
                                                                include_end_point = False,
                                                                allow_lane_change = allow_lane_change))
                prev_start = 0
            
        path_centerline.extend(self.get_centerline_section(last_lanelet, prev_start, dis_end))
        
        
        return path_centerline
        
    def get_centerline_section(self, lanelet, norm_dis_start, 
                            norm_dis_end, include_end_point = False, 
                            allow_lane_change = True):
        '''
            Given: a lanelet and a section of the lanelet defined by normalized start and end length along the centerline
            return a list of points that represent the centerline of the section
            
        ''' 
        centerline_section = []
        centerline = to2D(lanelet.centerline)
        length = lanelet2.geometry.length(centerline)
        speed_limit = self.get_lanelet_speed_limit(lanelet)
        
        assert norm_dis_start*norm_dis_end >= 0, "The start and end length must have the same sign"
        assert abs(norm_dis_start) < abs(norm_dis_end), f"The start distance {norm_dis_start} must be smaller than the end distance {norm_dis_end}"
        if norm_dis_start < 0:
            centerline = centerline.invert()
        dis_start = max(abs(norm_dis_start) * length, 0)
        dis_end = min(abs(norm_dis_end) * length, length)
        
        num_points = len(centerline) - 1*include_end_point
        
        # add start point
        arc_coord = ArcCoordinates()
        arc_coord.dis = 0.0
        arc_coord.length = dis_start
        pt_start = fromArcCoordinates(centerline, arc_coord)
        width_left, width_right = self.get_lane_width(pt_start, lanelet, allow_lane_change)
        centerline_section.append([pt_start.x, pt_start.y, width_left, width_right, speed_limit])
        
        for i in range(num_points):
            pt = centerline[i]
            pt_len = self.get_dis_from_point(centerline, pt.x, pt.y)
            if pt_len <= dis_start:
                continue
            elif pt_len >= dis_end:
                break
            width_left, width_right = self.get_lane_width(pt, lanelet, allow_lane_change)
            centerline_section.append([pt.x, pt.y, width_left, width_right, speed_limit])
            
        if include_end_point:
            arc_coord.length = dis_end
            pt_end = fromArcCoordinates(centerline, arc_coord)
            width_left, width_right = self.get_lane_width(pt, lanelet, allow_lane_change)
            centerline_section.append([pt_end.x, pt_end.y, width_left, width_right, speed_limit])
            
        return centerline_section
    
    def get_lane_width(self, point, lanelet, allow_lane_change = True):
        '''
        Given:
            point: a point in the lanelet
            lanelet: a lanelet
        return:
            lane_width: the width of the lane at the point
        '''
        left_bound = lanelet.leftBound
        right_bound = lanelet.rightBound
        if allow_lane_change:
            left_lanelet = self.routing_graph.left(lanelet)
            right_lanelet = self.routing_graph.right(lanelet)
            if left_lanelet is not None:
                left_bound = left_lanelet.leftBound
            if right_lanelet is not None:
                right_bound = right_lanelet.rightBound
                
        left_bound = to2D(left_bound)
        right_bound = to2D(right_bound)
        
        baisc_point = BasicPoint2d(point.x, point.y)
        
        width_left = lanelet2.geometry.distance(left_bound, baisc_point)
        width_right = lanelet2.geometry.distance(right_bound, baisc_point)
        return width_left, width_right
    
    @staticmethod
    def get_dis_from_point(line_string, x, y):
        '''
        Given:
            x: x coordinate 
            y: y coordinate 
        return:
            dis: distance from the start of the linestring to the point along the line string
        '''
        pt = BasicPoint2d(x, y)
        return toArcCoordinates(line_string, pt).length
    
    @staticmethod
    def get_lanelet_length(lanelet):
        '''
        This function returns the length of a lanelet's centerline
        '''
        return lanelet2.geometry.length2d(lanelet)
    
    @staticmethod
    def get_linestring_length(line_string):
        '''
        This function returns the length of a linestring
        '''
        return lanelet2.geometry.length(line_string)
    
    @staticmethod
    def get_lanelet_speed_limit(lanelet):
        '''
        This function returns the speed limit of a lanelet
        '''
        if 'speed' not in lanelet.attributes:
            return 2.0
        else:
            return float(lanelet.attributes['speed'])
    
    @staticmethod
    def get_point_at_dis(line_tring, dist):
        '''
        Returns the piecewise linearly interpolated point at the given distance.
        * @param lineString the lineString to iterate. Size must be >0.
        * @param dist distance along linestring. If negative, the lineString is
            iterated in reversed order.
        * @return The closest point.
        
        If the distance is greater length, the end point is returned (or start point
        if <0).
        '''
        return lanelet2.geometry.interpolatedPointAtDistance(line_tring, dist)
    
    @staticmethod
    def get_point_at_normalized_dis(line_tring, dist_norm):
        '''
        Returns the piecewise linearly interpolated point at the given normalized distance.
        * @param lineString the lineString to iterate. Size must be >0.
        * @param dist_norm normalized distance along linestring. If negative, the lineString is
            iterated in reversed order.
        * @return The closest point.
        
        If the distance is greater length, the end point is returned (or start point
        if <0).
        '''
        dis = dist_norm * lanelet2.geometry.length(line_tring)
        return lanelet2.geometry.interpolatedPointAtDistance(line_tring, dis)
    
    @staticmethod
    def project_xy_to_linestring(x, y, line_string):
        '''
        Given:
            x: x coordinate 
            y: y coordinate 
        return:
            dis: distance to the nearest lanelet. 0 if the point is inside the lanelet
            lanelet: the nearest lanelet
        '''
        pt = BasicPoint3d(float(x),float(y), 0.0)
        pt_project =  lanelet2.geometry.project(line_string, pt)
        return pt_project.x, pt_project.y
    
    def load_lanelet_map(self, osm_file):
        '''
        Load a Lanelet2::LaneletMap object from a file
        '''
        return lanelet2.io.load(osm_file,self.projector)
    
    def save_lanelet_map(self, lanelet_map, osm_file):
        '''
        Save a Lanelet2::LaneletMap object to a file
        '''
        lanelet2.io.write(osm_file, lanelet_map, self.projector)
        
        
    ''' Following are helper functions for lanelet2 objects '''

    def warmup(self):
        '''
        This function is used to warmup the lanelet2 library
        by cache all the centerlines of lanelets
        
        '''
        for lanelet in self.lanelet_layer:
            lanelet.centerline
