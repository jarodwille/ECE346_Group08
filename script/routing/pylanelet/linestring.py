from pyspline import Curve
import numpy as np

class LineType:
    WHITE_SOLID = 0
    WHITE_DASHED = 1
    DOUBLE_YELLOW = 2
    VIRTUAL_LINE = 3
    CENTER_LINE = 4

class PyLineString:
    def __init__(self, line_id: int, points: np.ndarray, line_type: LineType) -> None:
        '''
        Constructor for PyLineString
        :param id: ID of the line string
        :param points: Nx2 array of points (x,y) in the line string
        '''
        self.id = line_id
        self.points = points
        self.spline = Curve(x=points[:,0], y=points[:,1], k=3)
        self.length = self.spline.getLength()
        self.type = line_type
        
    def __str__(self) -> str:
        return f'PyLineString {self.id} with type {self.type} and length {self.length}'
        
    def sample_points(self, start: float = 0, end: float = 1, endpoint: bool=False, num: int = 100) -> np.ndarray:
        '''
        Uniformly sample n points along the line string between start and end
        :param start: normalized start position [0,1)
        :param end: normalized end position (0,1]
        :param n: number of points to sample
        :return: Nx2 array of points (x,y)
        '''
        sampled_s = np.linspace(start, end, num, endpoint=endpoint) 
        
        return self.spline.getValue(sampled_s)
    
    def project_points(self, points: np.ndarray) -> float:
        '''
        Project a point onto the line string
        :param points: 2D point (x,y) or a Nx2 array of points
        :return: closest point(s) on the line string (Nx2 array) 
        '''
        s, _ = self.spline.projectPoint(points)
        projected_points = self.spline.getValue(s)
        return projected_points
    
    def distance_to_point(self, point: np.ndarray) -> float:
        '''
        Compute distance between a point and the line string
        :param point: 2D point (x,y)
        :return: distance(s) to the line string
        '''
        point = point.astype(float)
        _, d = self.spline.projectPoint(point)
        return np.sqrt(d[0]**2 + d[1]**2)
    
    def get_ref_pose(self, s):
        '''
        Get reference pose (x,y,theta) at normalized position s
        :param s: normalized position [0,1]
        :return: reference pose (x,y,theta)
        '''
        
        x, y = self.spline.getValue(s)
        deri = self.spline.getDerivative(s)
        theta = np.arctan2(deri[1], deri[0])
        return np.array([x, y, theta])