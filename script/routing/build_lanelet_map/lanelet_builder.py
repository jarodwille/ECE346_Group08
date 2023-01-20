from ui import UI
from util import gen_linestring, gen_lanelet, load_lanelet_map, save_lanelet_map
import inquirer
import lanelet2
from lanelet2.core import LaneletMap
import numpy as np
import os
np.set_printoptions(precision=3, suppress=True)

class LaneletBuilder:
    def __init__(self, img_path, map_path, width, height) -> None:
        self.map_path = map_path
        self.ui = UI(img_path, width, height)
        self.lanelet_dict = {}
        self.linestring_dict = {}
        self.load_map()
        
    def load_map(self):
        if os.path.isfile(self.map_path):
            old_map = load_lanelet_map(self.map_path)
            
            # loop through linestrings
            linestring_layer = old_map.lineStringLayer
            for linestring in linestring_layer:
                self.linestring_dict[linestring.id] = linestring
                self.ui.plot_linestring(linestring, update=False)
            
            # loop through lanelets
            lanelet_layer = old_map.laneletLayer
            for lanelet in lanelet_layer:
                self.lanelet_dict[lanelet.id] = lanelet
                self.ui.plot_lanelet(lanelet, update=False)
            self.ui.update_plot()
            print("Loaded map from ", self.map_path)
        
    def choose_linestring_id(self):
        '''
        Return a linestring object by id from user input
        '''
        while True:
            id = input('Please enter the id of the linestring you want to choose: ')
            id = int(id)
            if id in self.linestring_dict:
                return self.linestring_dict[id]
            else:
                print(f'linstring {id} does not exist. Please try again.')
                
    def choose_linestring_from_lanelet(self):
        '''
        Choose either the left or right boundary of a lanelet by user input
        '''
        lanelet = self.choose_lanelet()
        q = [inquirer.List('q', 
                message='Is this linestring the left or right boundary of the lanelet?',
                choices=['left', 'right'])]
        answer = inquirer.prompt(q)
        if answer['q'] == 'left':
            return lanelet.leftBound
        else:
            return lanelet.rightBound
        
    def choose_lanelet(self):
        '''
        Return a lanelet object by id from user input
        '''
        while True:
            id = input('Please enter the id of the lanelet you want to choose: ')
            id = int(id)
            if id in self.lanelet_dict:
                return self.lanelet_dict[id]
            else:
                print(f'lanelet {id} does not exist. Please try again.')
        
    def choose_linestring(self):
        # Get an existing linestring 
        q = [inquirer.List('q', message='Do you want to choose a linestring by id or by lanelet?',
                            choices=['id', 'lanelet'])]
        
        if inquirer.prompt(q)['q'] == 'id':
            linestring = self.choose_linestring_id()
        else:
            linestring = self.choose_linestring_from_lanelet()
        print("Get linestring: ", linestring)
        # Decide if we want to invert the linestring
        q_inv = [inquirer.Confirm('q_inv', message='Do you want to invert the linestring?')]
        if inquirer.prompt(q_inv)['q_inv']:
            linestring = linestring.invert()
        return linestring
                
    def build_linestring(self):
        q_type = [inquirer.List('q_type', message='Do type of linestring?',
                                    choices=['solid', 'solid_solid', 'dashed', 'virtual'])]
        
        start_point = None
        end_point = None
        # Get the start point
        q_start = [inquirer.Confirm('q', message='Do you want to choose an existing start point?')]
        if inquirer.prompt(q_start)['q']:
            prev_linestring = self.choose_linestring()
            start_point = prev_linestring[-1]
            print("Get start point: ", start_point, " from the end of linestring: ", prev_linestring.id)
        
        q_end = [inquirer.Confirm('q', message='Do you want to choose an existing end point?')]
        if inquirer.prompt(q_end)['q']:
            next_linestring = self.choose_linestring()
            end_point = next_linestring[0]
            print("Get end point: ", end_point, " from the front of linestring: ", next_linestring.id)
        # get the type of linestring
        linestring_type = inquirer.prompt(q_type)['q_type']
        print("Start selecting points for the new linestring!")
        
        # collect points
        collected_xys = self.ui.collect_points()
        new_linestring = gen_linestring(collected_xys, start_point, end_point, linestring_type)
        self.ui.plot_linestring(new_linestring)
        # print("New linestring created: ", new_linestring)
        self.linestring_dict[new_linestring.id] = new_linestring
        return new_linestring
    
    def build_lanelet(self):
        right_linestring = None
        left_linestring = None
        q_linestring = [inquirer.Confirm('linestring', message='Build a new linestring?')]
        print("Creating RIGHT bound of a new lanelet")
        choice = inquirer.prompt(q_linestring)['linestring']
        
        if choice:
            right_linestring = self.build_linestring()
        else:
            right_linestring = self.choose_linestring()
            
        print("Creating LEFT bound of a new lanelet")
        choice = inquirer.prompt(q_linestring)['linestring']
        if choice:
            left_linestring = self.build_linestring()
        else:
            left_linestring = self.choose_linestring()
        
        new_lanelet = gen_lanelet(left_linestring, right_linestring)
        self.ui.plot_lanelet(new_lanelet)
        self.lanelet_dict[new_lanelet.id] = new_lanelet
        print("New lanelet created: ", new_lanelet)
        
    def remove_lanelet(self):
        '''
        Remove a lanelet object by id from user input
        '''
        id = input('Please enter the id of the lanelet you want to remove: ')
        id = int(id)
        if id in self.lanelet_dict:
            self.lanelet_dict.pop(id, None)
            self.ui.remove_lanelet(id)
        else:
            print(f'lanelet {id} does not exist. Please try again.')

    def save_map(self):
        map_to_save = LaneletMap()
        for lanelet in self.lanelet_dict.values():
            map_to_save.add(lanelet)
        save_lanelet_map(map_to_save, self.map_path)
        print("Map saved to: ", self.map_path)
        
    def run(self):
        q_main = [inquirer.List('main', message='What do you want to do?',
                                choices=['Add', 'Remove', 'Save', 'Quit'])]
        while True:
            choice = inquirer.prompt(q_main)['main']
            if choice == 'Quit':
                self.save_map()
                break
            elif choice == 'Remove':
                self.remove_lanelet()
            elif choice == 'Save':
                self.save_map()
            else:
                self.build_lanelet()


if __name__ == '__main__':
    img_path = 'IMG_0098.jpeg'
    map_path = 'track.osm'
    builder = LaneletBuilder(img_path, map_path, 6.05, 6.05)
    builder.run()