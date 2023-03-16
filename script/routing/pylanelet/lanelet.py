import pyspline
# import shapely
import numpy as np
from typing import Union
from .linestring import PyLineString


class PyLaneLet:
    def __init__(self, id: int, center_line: PyLineString, 
                left_boundary: PyLineString, right_boundary: PyLineString) -> None:
        
        # Lanelet ID
        self.id = id
        
        # road boundary and center line
        self.center_line = center_line
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary
        
        # lanelet length (of center line)
        self.length = self.center_line.length
        
        self.predecessor = []
        self.successor = []
        self.left = []
        self.right = []
        
        self.speed_limit = 2.0
        
    def get_section_width(self, start: float, end: float, endpoint: bool=False, num: int = 100):
        '''
        Get width of the lanelet section between start and end
        Parameters:
            start: float, normalized start position
            end: float, normalized end position
            endpoint: bool, whether to include end point
            num: int, number of points to sample
        Returns:
            width: float, width of the lanelet section
        '''
        left_vertices = self.left_boundary.sample_points(start, end, endpoint, num)
        right_vertices = self.right_boundary.sample_points(start, end, endpoint, num)
        
        return np.linalg.norm(left_vertices - right_vertices, axis=1)
    
    def get_section_vertices(self, start: float, end: float, endpoint: bool=False, num: int = 100):
        '''
        Get vertices of the lanelet section between start and end
        Parameters:
            start: float, normalized start position
            end: float, normalized end position
            endpoint: bool, whether to include end point
            num: int, number of points to sample
        Returns:
            polygon_vertices: (2*num, 2) array of vertices, 
                        first half is left boundary, second half is right boundary (in reverse order)
        '''
        left_vertices = self.left_boundary.sample_points(start, end, endpoint, num)
        right_vertices = self.right_boundary.sample_points(start, end, endpoint, num)
        
        right_vertices_flip = np.flip(right_vertices, axis=0)
        return np.concatenate((left_vertices, right_vertices_flip), axis=0)
        
    def get_section_centerline(self, start: float, end: float, endpoint: bool=False, num: int = 100):
        '''
        Get center line of the lanelet section between start and end
        Parameters:
            start: float, normalized start position
            end: float, normalized end position
            endpoint: bool, whether to include end point
            num: int, number of points to sample
        Returns:
            (num, 2) array of vertices of sampled center line
        '''
        return self.center_line.sample_points(start, end, endpoint, num)
        
    def add_successor(self, successors: Union[list, int]):
        '''
        Add successor lanelet ID to the current lanelet
        Parameters:
            successors: int or list of int
        '''
        if isinstance(successors, int):
            self.successor.append(successors)
        else:
            self.successor.extend(successors)
    
    def add_predecessor(self, predecessors: Union[list, int]):
        '''
        Add predecessor lanelet ID to the current lanelet
        Parameters:
            predecessors: int or list of int
        '''
        if isinstance(predecessors, int):
            self.predecessor.append(predecessors)
        else:
            self.predecessor.extend(predecessors)
    
    def add_left(self, left: Union[list, int]):
        '''
        Add left lanelet ID to the current lanelet
        Parameters:
            left: int or list of int
        '''
        if isinstance(left, int):
            self.left.append(left)
        else:
            self.left.extend(left)
            
    def add_right(self, right: Union[list, int]):
        '''
        Add right lanelet ID to the current lanelet
        Parameters:
            right: int or list of int
        '''
        if isinstance(right, int):
            self.right.append(right)
        else:
            self.right.extend(right)
            
    def is_successor(self, lanelet_id: int):
        '''
        Check if the current lanelet is the successor of the lanelet with the given ID
        Parameters:
            lanelet_id: int
        '''
        return lanelet_id in self.successor

    def is_predecessor(self, lanelet_id: int):
        '''
        Check if the current lanelet is the predecessor of the lanelet with the given ID
        Parameters:
            lanelet_id: int
        '''
        return lanelet_id in self.predecessor
    
    def is_left(self, lanelet_id: int):
        '''
        Check if the current lanelet is the left lanelet of the lanelet with the given ID
        Parameters:
            lanelet_id: int
        '''
        return lanelet_id in self.left
    
    def is_right(self, lanelet_id: int):
        '''
        Check if the current lanelet is the right lanelet of the lanelet with the given ID
        Parameters:
            lanelet_id: int
        '''
        return lanelet_id in self.right
    
    def distance_to_centerline(self, point: np.ndarray):
        '''
        Get distance from the given point to the center line
        Parameters:
            point: (2,) array
        Returns:
            distance: float
        '''
        # This d is (curve(s) - point)
        s, d = self.center_line.spline.projectPoint(point)
        
        sampled_pt = self.center_line.spline.getValue(s)
        deri = self.center_line.spline.getDerivative(s)
        slope = np.arctan2(deri[1], deri[0]) # [N,]
        return -np.sin(slope) * d[0] + np.cos(slope) * d[1]
        
    
        
        
        