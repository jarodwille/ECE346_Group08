import lanelet2
from lanelet2.core import (BasicPoint2d, BasicPoint3d, 
                            BoundingBox2d, Lanelet, LaneletMap,
                            LaneletWithStopLine, LineString2d, LineString3d, Point2d, Point3d,
                            RightOfWay, TrafficLight, getId)
from lanelet2.projection import (UtmProjector, MercatorProjector,
                                    LocalCartesianProjector, GeocentricProjector)
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

'''
This class is a wrapper for lanelet2 objects
It uses the lanelet2 python API to provide a more convenient interface with gerneic data types
'''
class LaneletWrapper:
    def __init__(self, osm_file) -> None:
        
        self.projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
        
        
        # load lanelet map
        self.lanelet_map = self.load_lanelet_map(osm_file)
        self.lanelet_layer = self.lanelet_map.laneletLayer
        # create routing graph
        self.traffic_rules = lanelet2.traffic_rules.create(
                                lanelet2.traffic_rules.Locations.Germany, 
                                lanelet2.traffic_rules.Participants.Vehicle
                            )
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules)
        
        self.warmup()
        pass
    
    def find_lanelet_by_xy(self, x, y):
        '''
        Given:
            x: x coordinate 
            y: y coordinate 
        return:
            dis: distance to the nearest lanelet. 0 if the point is inside the lanelet
            lanelet: the nearest lanelet
        '''
        pt = BasicPoint2d(float(x),float(y))
        return lanelet2.geometry.findNearest(self.lanelet_layer, pt, 1)[0]
    
    @staticmethod
    def get_lanelet_length(lanelet):
        '''
        This function returns the length of a lanelet's centerline
        '''
        return lanelet2.geometry.length2d(lanelet)
    
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
