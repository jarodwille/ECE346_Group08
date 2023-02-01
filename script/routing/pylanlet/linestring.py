from pyspline import Curve
import numpy as np

class LineType:
    WHITE_SOLID = 0
    WHITE_DASHED = 1
    DOUBLE_YELLOW = 2
    VIRTUAL_LINE = 3
    CENTER_LINE = 4

class LineString:
    def __init__(self, line_id: int, points: np.ndarray, line_type: LineType) -> None:
        '''
        Constructor for LineString
        :param id: ID of the line string
        :param points: Nx2 array of points (x,y) in the line string
        '''
        self.id = line_id
        self.points = points
        self.spline = Curve(x=points[:,0], y=points[:,1], k=3)
        self.length = self.spline.getLength()
        self.type = line_type
        
    def __str__(self) -> str:
        return f'LineString {self.id} with type {self.type} and length {self.length}'
        
    def sample_points(self, start: float = 0, end: float = 1, n: int = 100) -> np.ndarray:
        '''
        Uniformly sample n points along the line string between start and end
        :param start: normalized start position [0,1)
        :param end: normalized end position (0,1]
        :param n: number of points to sample
        :return: Nx2 array of points (x,y)
        '''
        sampled_s = np.linspace(start, end, n) 
        
        return self.spline.getValue(sampled_s)
    
    def project_point(self, point: np.ndarray) -> float:
        '''
        Project a point onto the line string
        :param point: 2D point (x,y) or a Nx2 array of points
        :return: closest point(s) on the line string (Nx2 array) 
        '''
        s, _ = self.spline.projectPoint(point)
        projected_point = self.spline.getValue(s)
        return projected_point