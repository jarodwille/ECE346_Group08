import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from .lanelet import PyLaneLet

class PyLaneletMap:
    def __init__(self) -> None:
        self.lanelets = {}
        self.routing_graph = nx.DiGraph()
        self.graph_initialized = False
        self.psi_weight = 1 # weight for heading difference in cost function
        
        
    def add_lanelet(self, lanelet: PyLaneLet):
        self.lanelets[lanelet.id] = lanelet
        
    def get_lanelet(self, lanelet_id: int) -> PyLaneLet:
        return self.lanelets[lanelet_id]
    
    
    def get_closest_lanelet(self, pose, check_psi: bool=False) -> PyLaneLet:
        '''
        Get the closest lanelet to the given pose
        Parameters:
            pose: array, [x, y, (optional) psi]
            check_psi: bool, whether to check heading difference
        Returns:
            closest_lanelet: PyLaneLet, closest lanelet
            lanelet_s: float, normalized position on the centerline
            
        '''
        pose = np.array(pose, dtype=np.float32)
        min_dist = float('inf')
        closest_lanelet = None
        lanelet_s = None
        point = pose[:2]
        if check_psi:
            psi = pose[2]
        
        for lanelet in self.lanelets.values():
            spline = lanelet.center_line.spline
            s, d = spline.projectPoint(point)
            dist = np.sqrt(d[0]**2 + d[1]**2)
            
            # check heading
            if check_psi:
                deriv = spline.getDerivative(s)
                ref_psi = np.arctan2(deriv[1], deriv[0])
                psi_dist = psi - ref_psi
                psi_dist = self.psi_weight * np.abs(np.arctan2(np.sin(psi_dist), np.cos(psi_dist)))
            else: 
                psi_dist = 0
            
            total_dist = dist + psi_dist
            
            if total_dist < min_dist:
                min_dist = total_dist
                closest_lanelet = lanelet
                lanelet_s = s
        return closest_lanelet, lanelet_s
    
    def build_graph(self, lane_change_cost = 0.5):
        '''
        Build networkx graph for routing
        '''
        print("Building routing graph with lane change cost: ", lane_change_cost)
        for lanelet in self.lanelets.values():
            cur_id = lanelet.id
            # lane change
            for left_id in lanelet.left:
                self.routing_graph.add_edge(cur_id, left_id, weight=lane_change_cost)
            for right_id in lanelet.right:
                self.routing_graph.add_edge(cur_id, right_id, weight=lane_change_cost)
            # lane follow
            for successor_id in lanelet.successor:
                self.routing_graph.add_edge(cur_id, successor_id, weight=lanelet.length)
        
        self.graph_initialized = True
            
    def get_shortest_route(self, start_id: int, end_id: int):
        '''
        Get the shortest route from start_id to end_id
        '''
        route = nx.shortest_path(self.routing_graph, start_id, end_id, weight='weight')
        route_length = 0
        for i in range(len(route)-1):
            route_length += self.routing_graph[route[i]][route[i+1]]['weight']
        
        return route, route_length
    
    def get_shortest_path(self, start, end, start_pose: bool = False, end_pose: bool = False):
        '''
        Get the shortest path from start to end
        '''
        if not self.graph_initialized:
            self.build_graph()
            
        centerline_list = []
        start_lanelet, start_s = self.get_closest_lanelet(start, check_psi=start_pose)
        end_lanelet, end_s = self.get_closest_lanelet(end, check_psi=end_pose)
        
        route = None
        
        # check if start and end are in the same lanelet or they are neighbors
        if (start_lanelet.id == end_lanelet.id \
                or end_lanelet.id in start_lanelet.left \
                or end_lanelet.id in start_lanelet.right):
            if abs(start_s - end_s)*start_lanelet.length < 0.1:
                print("Start and end points are too close")
                return None
            if end_s < start_s:
                centerline_list.append(
                    self.get_reference(start_lanelet, start_s, 1, endpoint = False)
                    )
                route_cost = float('inf')
                # search through successors for the best route
                for successor_id in start_lanelet.successor:
                    temp_route, temp_route_cost = self.get_shortest_route(successor_id, end_lanelet.id)
                    if temp_route_cost < route_cost:
                        route = temp_route
                        route_cost = temp_route_cost
                # new start point is the beginning of the best successor 
                start_s = 0
        # Get the shortest route if we have not found one yet
        if route is None:
            route, _ = self.get_shortest_route(start_lanelet.id, end_lanelet.id)
            
        num_lanelets = len(route)
        for i in range(num_lanelets-1):
            cur_lanelet = self.lanelets[route[i]]
            next_lanelet_id = route[i+1]
            # check if we need to change lane
            if next_lanelet_id in cur_lanelet.left or next_lanelet_id in cur_lanelet.right:
                if i == num_lanelets-2:
                    # change lane to the last lanelet
                    cur_end = (end_s-start_s)*0.3 + start_s
                    next_start = (end_s - start_s)*0.7 + start_s
                else:
                    cur_end = (1-start_s)*0.3 + start_s
                    next_start = (1 - start_s)*0.7 + start_s
            else:
                cur_end = 1
                next_start = 0
            centerline_list.append(
                self.get_reference(cur_lanelet, start_s, cur_end, endpoint = False)
                )
            start_s = next_start
        
        centerline_list.append(
            self.get_reference(end_lanelet, start_s, end_s, endpoint = True)
            )
        
        return np.concatenate(centerline_list, axis=0)
    
    def plot_map(self):
        plt.figure(figsize=(10, 10))
        plotted_linestring = []
        for _, lanelet in self.lanelets.items():
            if lanelet.left_boundary.id not in plotted_linestring:
                points = lanelet.left_boundary.sample_points(0, 1, True)
                plt.plot(points[:, 0], points[:, 1], 'k')
                plotted_linestring.append(lanelet.left_boundary.id)
            if lanelet.right_boundary.id not in plotted_linestring:
                points = lanelet.right_boundary.sample_points(0, 1, True)
                plt.plot(points[:, 0], points[:, 1], 'k')
                plotted_linestring.append(lanelet.right_boundary.id)
                
        plt.axis('equal')
        
    def get_reference(self, lanelet, start_s, end_s, endpoint: bool = False):
        '''
        Get the reference line of a lanelet 
        Parameters:
            lanlet: the lanelet
            start_s: the start s value
            end_s: the end s value
        return:
            reference line:[Nx5], [x,y,left_width,right_width,speed_limit]
        '''
        center_line = lanelet.get_section_centerline(start_s, end_s, endpoint = endpoint)
        
        cur_width = lanelet.get_section_width(start_s, end_s, endpoint = endpoint)
        left_width = cur_width/2
        right_width = cur_width/2
        
        for left_id in lanelet.left:
            left_lanelet = self.lanelets[left_id]
            left_width += left_lanelet.get_section_width(start_s, end_s, endpoint = endpoint)
        
        for right_id in lanelet.right:
            right_lanelet = self.lanelets[right_id]
            right_width += right_lanelet.get_section_width(start_s, end_s, endpoint = endpoint)
        
        left_width = left_width[:, np.newaxis]
        right_width = right_width[:, np.newaxis]
        speed_limit = lanelet.speed_limit*np.ones_like(left_width)
        
        reference = np.concatenate([center_line, left_width, right_width, speed_limit], axis=1)
        return reference