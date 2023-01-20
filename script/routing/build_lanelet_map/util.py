import lanelet2
from lanelet2.core import (getId, Point3d, LineString3d,
                            Lanelet)

from lanelet2.projection import LocalCartesianProjector

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
def gen_point(pt):
    '''
    Generate a Lanelet2::Point2d object from a tuple/list/nparray of 2d coordinates
    '''
    return Point3d(getId(), float(pt[0]), float(pt[1]), 0.0)


def gen_linestring(pts, start_point = None, end_point = None, line_type = 'dashed'):
    '''
    Generate a Lanelet2::LineString2d object from a list of tuples/lists/nparrys of 2d coordinates
    If start_point and end_point are not None, they will be used as the start and end points of the linestring
    
    https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LinestringTagging.md
    for line_type options
    '''
    d,n = pts.shape
    assert d == 2, 'pts should be a [2,n] array'
    line_string = LineString3d(getId(),[])
    if line_type == 'virtual':
        line_string.attributes['type'] = 'virtual'
    else:
        line_string.attributes['type'] = 'line_thin'
        line_string.attributes['subtype'] = line_type
    for i in range(n):
        point = gen_point(pts[:,i])
        if i == 0:
            if start_point is None:
                start_point = point
            else:
                line_string.append(start_point)
                
        line_string.append(point)
        
        if i==n-1:
            if end_point is None:
                end_point = point
            else:
                line_string.append(end_point)
        
    return line_string

def gen_lanelet(left_line, right_line):
    '''
    Generate a Lanelet2::Lanelet object from a list of tuples/lists/nparrys of 2d coordinates
    '''
    lanelet = Lanelet(getId(), left_line, right_line)
    lanelet.attributes['type'] = 'lanelet'
    lanelet.attributes['subtype'] = 'road'
    return lanelet

def save_lanelet_map(lanelet_map, filename):
    '''
    Save a Lanelet2::LaneletMap object to a file
    '''
    projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
    lanelet2.io.write(filename, lanelet_map, projector)
    
def load_lanelet_map(filename):
    '''
    Load a Lanelet2::LaneletMap object from a file
    '''
    projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))
    return lanelet2.io.load(filename, projector)
    
