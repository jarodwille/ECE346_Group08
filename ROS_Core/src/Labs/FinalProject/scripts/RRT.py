import numpy as np
import matplotlib.pyplot as plt

def rrt(start, goal, obstacles, max_iter=1000, max_dist=0.1):
    # Initialize the tree with the start node
    nodes = [start]
    edges = []
    
    for i in range(max_iter):
        # Randomly sample a point in the space
        sample = np.random.rand(2) * np.array([20, 20]) - np.array([10, 10])
        
        # Find the nearest node in the tree to the sampled point
        nearest_idx = np.argmin(np.linalg.norm(np.array(nodes) - sample, axis=1))
        nearest_node = nodes[nearest_idx]
        
        # Calculate the vector to the sampled point and normalize it
        v = sample - nearest_node
        v_norm = v / np.linalg.norm(v)
        
        # Calculate the new point by adding a scaled vector to the nearest node
        new_node = nearest_node + max_dist * v_norm
        
        # Check if the new node collides with any obstacles
        if any(np.linalg.norm(np.array(obstacles) - new_node, axis=1) < 0.1):
            continue
        
        # Add the new node to the tree and record the edge
        nodes.append(new_node)
        edges.append((nearest_idx, len(nodes) - 1))
        
        # Check if the goal is reached
        if np.linalg.norm(new_node - goal) < 0.1:
            goal_idx = len(nodes) - 1
            break
    
    # If the goal was not reached, return an empty path
    if not 'goal_idx' in locals():
        return []
    
    # Build the path by tracing back through the edges
    path = [goal]
    curr_idx = goal_idx
    while curr_idx != 0:
        prev_idx = [e[0] for e in edges if e[1] == curr_idx][0]
        path.append(nodes[prev_idx])
        curr_idx = prev_idx
    path.append(start)
    path.reverse()
    
    return path
