import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Cursor
import random
import imageio.v2 as imio
import time

class UI:
    def __init__(self, img_path, width, height):
        self.width = width
        self.height = height
        self.lanelet_patch = {}
        plt.ion()
        
        self.load_image(img_path)
        self.cursor = Cursor(self.ax, color='green', linewidth=1, useblit=True)
        
    def load_image(self, path):
        print("Loading image from {}".format(path))
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(12, 8)
        img = imio.imread(path)
        self.im_height, self.im_width, _ = img.shape
        self.ax.imshow(img)
        self.ax.get_xaxis().set_visible(False)
        self.ax.get_yaxis().set_visible(False)
        plt.tight_layout()
        self.update_plot()
        

    def collect_points(self):
        '''
        Returns:
            points: 2xN array, pixel coordinates
            xys: 2xN array, real coordinates
        '''
        points = self.fig.ginput(-1, timeout=0, show_clicks=True)
        points = np.array(points).T #[2xN] array
        xys = self.point2xy(points)
        return xys

    def point2xy(self, points):
        # points: 2xN array
        # xys: 2xN array
        x = points[0,:]/self.im_width*self.width
        y = (1.0 - points[1,:]/self.im_height)*self.height
        return np.array([x, y])

    def xy2point(self, xys):
        # xys: 2xN array
        # points: 2xN array
        i = xys[0,:]/self.width*self.im_width
        j = (1.0 - xys[1,:]/self.height)*self.im_height
        return np.array([i, j])
    
    def plot_lanelet(self, lanelet, update = True):
        # get polygon of lanelet
        lanelet_polygon = lanelet.polygon2d()
        num_points = len(lanelet_polygon)
        vertices_xy = np.zeros((2, num_points+1))
        for i in range(num_points):
            vertices_xy[0,i] = lanelet_polygon[i].x
            vertices_xy[1,i] = lanelet_polygon[i].y
        vertices_xy[:,num_points] = vertices_xy[:,0]
        
        vertices_ij = self.xy2point(vertices_xy)
        fillcolor = (random.random(), random.random(), random.random())

        patch = Polygon(vertices_ij.T, closed=True, fill=True,
                        alpha=0.5, 
                        # label='lanelet'+str(lanelet.id), 
                        facecolor = fillcolor)
        
        self.lanelet_patch[lanelet.id] = patch
        self.ax.add_patch(patch)
        if update:
            self.update_plot()
    
    def update_plot(self):
        self.ax.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def plot_linestring(self, linestring, update = True):
        num_points = len(linestring)
        point_xy = np.zeros((2, num_points))
        
        label = 'linestring'+str(linestring.id)
        linewidth = 1
        
        if linestring.attributes['type'] == 'virtual':
            line_type = ':o'
        elif linestring.attributes['subtype'] == 'dashed':
            line_type = '--o'
        elif linestring.attributes['subtype'] == 'solid_solid':
            line_type = '-.o'
        else:
            line_type = '-o'
        for i in range(num_points):
            point_xy[0,i] = linestring[i].x
            point_xy[1,i] = linestring[i].y
        point_ij = self.xy2point(point_xy)
        self.ax.plot(point_ij[0,:], point_ij[1,:], line_type, 
                    linewidth=linewidth, label=label)
        if update:
            self.update_plot()
        
    def remove_lanelet(self, lanelte_id):
        patch = self.lanelet_patch[lanelte_id]
        patch.remove()
        self.lanelet_patch.pop(lanelte_id, None)
        self.update_plot()
