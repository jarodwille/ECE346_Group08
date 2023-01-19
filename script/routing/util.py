import lanelet2
from lanelet2.core import (getId, Point3d, LineString3d,
                            Lanelet)

from lanelet2.projection import LocalCartesianProjector

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
    assert d == 2, "pts should be a [n,2] array"
    line_string = LineString3d(getId(),[])
    line_string.attributes["type"] = "line_thin"
    line_string.attributes["subtype"] = line_type
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
        
    return line_string, start_point, end_point

def gen_lanelet(left_line, right_line):
    '''
    Generate a Lanelet2::Lanelet object from a list of tuples/lists/nparrys of 2d coordinates
    '''
    lanelet = Lanelet(getId(), left_line, right_line)
    lanelet.attributes["type"] = "road"
    lanelet.attributes["subtype"] = "road"
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