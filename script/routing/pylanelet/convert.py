import numpy as np
from .linestring import PyLineString, LineType
from .lanelet import PyLaneLet
from .lanelet_map import PyLaneletMap
import matplotlib.pyplot as plt
import pickle

try:
    import lanelet2
    from lanelet2.core import BasicPoint2d, BasicPoint3d
    from lanelet2.projection import LocalCartesianProjector
    from lanelet2.geometry import to2D, toArcCoordinates, fromArcCoordinates, ArcCoordinates
except ImportError:
    print("Lanelet2 is not installed.")
    
    
class Lanelet2Converter:
    def __init__(self, osm_file) -> None:
        
        self.projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
        
        # load lanelet map
        self.lanelet_map = self.load_lanelet2_map(osm_file)
        self.lanelet_layer = self.lanelet_map.laneletLayer
        
        # create routing graph
        self.traffic_rules = lanelet2.traffic_rules.create(
                                lanelet2.traffic_rules.Locations.Germany, 
                                lanelet2.traffic_rules.Participants.Vehicle
                            )
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, self.traffic_rules)
        
        self.pylanelet_map = PyLaneletMap()
        
        
    
    def load_lanelet2_map(self, map_file: str) -> None:
        '''
        Load lanelet2 map from a map file
        Parameters:
            map_file: str, path to the map file
        '''
        return lanelet2.io.load(map_file, self.projector)
        
    def convert_lanelet2(self):
        for lanelet in self.lanelet_layer:
            id = lanelet.id
            # center line
            center_line = self.linestring_to_pylinstring(lanelet.centerline)
            left_boundary = self.linestring_to_pylinstring(lanelet.leftBound)
            right_boundary = self.linestring_to_pylinstring(lanelet.rightBound)
            
            pylanelet = PyLaneLet(id, center_line, left_boundary, right_boundary)
            
            # set speed limit
            if "speed" in lanelet.attributes:
                pylanelet.speed_limit = float(lanelet.attributes["speed"])
            
            # add relations
            predcessors = self.routing_graph.previous(lanelet, False)
            
            if predcessors:
                for predcessor in predcessors:
                    pylanelet.add_predecessor(predcessor.id)
                
            successors = self.routing_graph.following(lanelet, False)
            if successors:
                for successor in successors:
                    pylanelet.add_successor(successor.id)
            
            lefts = self.routing_graph.lefts(lanelet)
            if lefts:
                for left in lefts:
                    pylanelet.add_left(left.id)
                    
            rights = self.routing_graph.rights(lanelet)
            if rights:
                for right in rights:
                    pylanelet.add_right(right.id)
            
            self.pylanelet_map.add_lanelet(pylanelet)
            
    @staticmethod
    def linestring_to_pylinstring(linestring) -> PyLineString:
        '''
        Convert lanelet2 linestring to PyLineString
        Parameters:
            linestring: lanelet2.core.Linestring3d
        Returns:
            PyLineString
        '''
        id = linestring.id
        
        if "type" not in linestring.attributes:
            type = LineType.CENTER_LINE
        elif linestring.attributes["type"] == "virtual":
            type = LineType.VIRTUAL_LINE
        elif linestring.attributes["subtype"] == "solid_solid":
            type = LineType.DOUBLE_YELLOW
        elif linestring.attributes["subtype"] == "dashed":
            type = LineType.WHITE_DASHED
        elif linestring.attributes["subtype"] == "solid":
            type = LineType.WHITE_SOLID
        else:
            type = LineType.VIRTUAL_LINE
        
        points = []
        for point in linestring:
            if len(points) > 0:
                prev_pt = points[-1]
                dx = point.x - prev_pt[0]
                dy = point.y - prev_pt[1]
                dis = np.sqrt(dx**2 + dy**2)
                if dis > 0.2:
                    x_interp = np.linspace(prev_pt[0], point.x, int(dis/0.1), endpoint=True)
                    y_interp = np.linspace(prev_pt[1], point.y, int(dis/0.1), endpoint=True)
                    for i in range(1, len(x_interp)):
                        points.append([x_interp[i], y_interp[i]])
                    continue    
            points.append([point.x, point.y])
        points = np.array(points)
        return PyLineString(id, points, type)
    
    def plot_map(self):
        plt.figure(figsize=(10, 10))
        plotted_linestring = []
        for _, lanelet in self.pylanelet_map.lanelets.items():
            if lanelet.left_boundary.id not in plotted_linestring:
                points = lanelet.left_boundary.sample_points(0, 1, True)
                plt.plot(points[:, 0], points[:, 1], 'k')
                plotted_linestring.append(lanelet.left_boundary.id)
            if lanelet.right_boundary.id not in plotted_linestring:
                points = lanelet.right_boundary.sample_points(0, 1, True)
                plt.plot(points[:, 0], points[:, 1], 'k')
                plotted_linestring.append(lanelet.right_boundary.id)
                
        plt.axis('equal')
        
    def save_map(self, filename: str):
        with open(filename, 'wb') as f:
            pickle.dump(self.pylanelet_map, f, pickle.HIGHEST_PROTOCOL)
            
            

    