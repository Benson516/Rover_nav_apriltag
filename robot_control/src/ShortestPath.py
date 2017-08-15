#!/usr/bin/python

import numpy as np
import Queue as Q
from math import floor

def idx2metric(idx,x_spacing,y_spacing):
    """
    Transform the index (i,j) to metric (x,y)
    """
    return np.array([(idx[1] + 0.5)*x_spacing,(idx[0] + 0.5)*y_spacing]) # return (x,y)

def metric2idx(metric,x_spacing,y_spacing):
    """
    Transform the metric (x,y) to a "nearest" index (i,j)
    """
    return np.array( [int(floor(metric[1]/y_spacing)), int(floor(metric[0]/x_spacing))] ) # return (i,j)

def idx2D_2_idx1D(idx_2D,m,n):
    return (n*idx_2D[0] + idx_2D[1]) # idx_1D

def idx1D_2_idx2D(idx_1D,m,n):
    return divmod(idx_1D,n) # (i,j)

def get_neighbor_idx_weight(idx_u,m,n,x_spacing,y_spacing,occ_map):
    # (i,j,weight)
    idx_up = (idx_u[0] - 1, idx_u[1], y_spacing)
    idx_down = (idx_u[0] + 1, idx_u[1], y_spacing)
    idx_left = (idx_u[0], idx_u[1] - 1, x_spacing)
    idx_right = (idx_u[0], idx_u[1] + 1, x_spacing)
    # idx_neighbor_all = [idx_up,idx_down,idx_left,idx_right]
    idx_neighbor_all = [idx_down,idx_up,idx_left,idx_right]
    #
    idx_neighbor = list()
    for idx_n in idx_neighbor_all:
        if idx_n[0] >= 0 and idx_n[0] < m and idx_n[1] >= 0 and idx_n[1] < n:
            if occ_map[idx_n[0],idx_n[1]] == 0: # empty
                idx_neighbor.append(idx_n)
    return idx_neighbor

def relax():
    """
    The relax function used in Dijkstra algorithm for updating the distance between s (start) and v (a vertex)
    """
    pass

def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position
    Output:
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # Metric to index
    start_idx = metric2idx(start, x_spacing, y_spacing);
    goal_idx = metric2idx(goal, x_spacing, y_spacing);
    print "start_idx: ", start_idx, "goal_idx:", goal_idx

    # Get the map size
    m,n = occupancy_map.shape
    occupancy_map[goal_idx[0], goal_idx[1]] = 0; # change the goal position to free space

    # Initialize the dist matrix, max_value indicates infinity
    max_value = 10*(m*y_spacing + n*x_spacing)
    dist = (np.ones_like(occupancy_map))*max_value; # max_value: infinity

    # Initialize the distance for start position
    dist[start_idx[0], start_idx[1]] = 0.0; # The distance from start to start is 0

    # Create the list of parents
    prev = (np.ones_like(occupancy_map))*(-1.0); # -1: null
    # prev[start_idx[0], start_idx[1]] = -1

    # Make a min-heap
    heap = Q.PriorityQueue()
    for i in range(m):
        for j in range(n):
            if occupancy_map[i,j] == 0:
                heap.put_nowait(( dist[i,j], (i,j) ))

    # Iteration
    while not heap.empty():
        u = heap.get_nowait()
        u_dist = u[0]
        u_idx = u[1]
        while u_dist != dist[u_idx[0], u_idx[1]] and (not heap.empty()):
            u = heap.get_nowait()
            u_dist = u[0]
            u_idx = u[1]
        if heap.empty():
            break
        # print "u_idx: ", u_idx, "u_dist:", u_dist
        #
        v_list = get_neighbor_idx_weight(u_idx,m,n,x_spacing,y_spacing,occupancy_map)
        for v in v_list:
            v_idx = v[0:2]
            v_weight = v[2]
            # Relaxation
            if dist[v_idx[0], v_idx[1]] > (dist[u_idx[0], u_idx[1]] + v_weight):
                dist[v_idx[0], v_idx[1]] = (dist[u_idx[0], u_idx[1]] + v_weight)
                prev[v_idx[0], v_idx[1]] = idx2D_2_idx1D(u_idx,m,n)
                # ChangePriority(heap, v, dist[v])
                heap.put_nowait( (dist[v_idx[0], v_idx[1]], v_idx) )
    print "Dijkstra finished"
    print "Distance from start to goal:",dist[goal_idx[0], goal_idx[1]]
    print dist
    # Generate the path
    if dist[goal_idx[0], goal_idx[1]] == max_value: # not reachable
        print "The goal is not reachable from start."
        return np.array([])
    # If the goal is reachable
    current_idx = goal_idx;
    path = list()
    path.append(np.array([goal[0,0],goal[1,0]]))
    while True:
        prev_idx1D = prev[current_idx[0], current_idx[1]]
        if prev_idx1D >= 0:
            current_idx = idx1D_2_idx2D(prev_idx1D,m,n)
            path.append(idx2metric(current_idx, x_spacing, y_spacing))
        else:
            break
    path.append(np.array([start[0,0],start[1,0]]))
    # print "path:",path
    path_1 = np.array(path[::-1])
    print "The path:\n",path_1
    return path_1

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
        print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
        print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    fhandle = open('result.txt','w')
    # fhandle.truncate()

    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)
    fhandle.write(str(s))
    fhandle.write(' ')

    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)
    fhandle.write(str(s))
    # fhandle.write('\t')
    fhandle.close()

def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    # main()
    # test()
    test_for_grader()
