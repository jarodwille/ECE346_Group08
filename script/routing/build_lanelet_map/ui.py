import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imio


class UI:
    def __init__(self, img_path, width, height):
        self.width = width
        self.height = height
        self.load_image(img_path)
        self.lanelet_polygons = {}
        
    def load_image(self, path):
        print("Loading image from {}".format(path))
        img = imio.imread(path)
        self.im_h, self.im_w, _ = img.shape
        plt.imshow(img)
        plt.gca().get_xaxis().set_visible(False)
        plt.gca().get_yaxis().set_visible(False)
        plt.show(block=False)
         
    def collect_points(self):
        '''
        Returns:
            points: Nx2 array, pixel coordinates
            xys: Nx2 array, real coordinates
        '''
        points = plt.ginput(-1, timeout=0, show_clicks=True)
        points = np.array(points).T #[2xN] array
        xys = self.point2xy(points)
        return xys

    def point2xy(self, points):
        # points: Nx2 array
        # xys: Nx2 array
        x = points[0,:]/self.im_width*self.width
        y = (1.0 - points[1,:]/self.im_height)*self.height
        return np.array([x, y])

    def xy2point(self, xys):
        # xys: Nx2 array
        # points: Nx2 array
        i = xys[0,:]/self.width*self.im_width
        j = (1.0 - xys[1,:]/self.height)*self.im_height
        return np.array([i, j])
    
    def plot_lanelet(self, lanelet):
        pass
    
    def remove_lanelet(self, lanelet):
        pass


if __name__ == '__main__':
    img_path = '/Users/zixuz/Documents/GitRepo/-PrincetonRaceCar_routing/script/track_creation_matlab/IMG_0098.jpeg'
    ui = UI(img_path, 6.05, 6.05)
    