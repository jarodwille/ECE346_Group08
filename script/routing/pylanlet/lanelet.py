import pyspline
import shapely
import numpy as np


class LaneLet:
    def __init__(self, id) -> None:
        # Lanelet ID
        self.id = id
        
        # road boundary and center line
        self.center_line = None
        self.left_boundary = None
        self.right_boundary = None
        
        # lanelet length (of center line)
        self.length = 0
        
        
        
        
        