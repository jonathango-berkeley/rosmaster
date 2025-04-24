import math
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

#########################################################

#global parameter
buffer = 5    #size of buffer zone

#########################################################

class Node:
    def __init__(self, y, x):
        self.x = x
        self.y = y
        self.type = 100    #types: 0-100 [int] (from occupancy grid)
        self.start_dis = float('inf')    #distance to start node
        self.end_dis = float('inf')    #distance to end node

#########################################################

def dis_curr_end(curr, end):    #minimal distance from current node to end node
    
    x_dis = abs(end.x - curr.x)
    y_dis = abs(end.y - curr.y)

    #uncommented if diagonal movement is allowed 
    diag = min(x_dis, y_dis)    #minimal diagonal steps needed to reach end
    strai = x_dis + y_dis - 2 * diag    #minimal straight steps needed to reach end
    
    dis = math.sqrt(2) * diag + strai

    #if diagonal movement is not allowed
    #dis = x_dis + y_dis
    
    return dis

#########################################################

def min_end_dis(next_nodes):
    
    next_node = None
    
    for i in range(len(next_nodes)):
        
        node = next_nodes[i]
        
        if next_node == None or node.end_dis < next_node.end_dis:    #set next_node to the node with the smallest distance to end
            next_node = node

    return next_node

#########################################################

def get_nei(grid, curr):
    rows = len(grid)    #determine number of rows
    columns = len(grid[0])    #determine number of columns
    #add diagonal movement if allowed
    direc = np.array([[1, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [1, 1], [1, -1], [-1, -1]])    #possible directions (vertical, horizontal and #diagonal)
    
    nei_nodes = []    #list where neighbor nodes are added
    
    for j in range(len(direc)):
        
        direc_j = direc[j,:]
        
        nei_x = curr.x + direc_j[1]
        nei_y = curr.y + direc_j[0]

        if nei_x >= 0 and nei_y >= 0 and nei_x < columns and nei_y < rows:    #adds neighbor for every direction if in domain
            nei_nodes.append(grid[nei_x][nei_y])

    return nei_nodes

#########################################################

def dis_curr_nei(curr, nei_node):
    dis = math.sqrt(((curr.x - nei_node.x)**2) + ((curr.y - nei_node.y)**2))    #distance between two nodes
    
    #penalty if direction changes
    '''
    if prev_node:
        prev_dir = (curr.x - prev_node.x, curr.y - prev_node.y)
        curr_dir = (nei_node.x - curr.x, nei_node.y - curr.y)
        if curr_dir != prev_dir:
            dis = dis + 1    #add penalty if direction changes
    '''
    return dis

#########################################################

def find_path(grid, came_from, curr):    #reconstructs the optimal path from came_from
    path = []
    key = str(curr.x) + ' ' + str(curr.y)
    while key in came_from:
        
        path.insert(0, curr)
        curr = came_from[key]
        key = str(curr.x) + ' ' + str(curr.y)
        
    return path

#########################################################

def process_occupancy_grid(occupancy_grid_msg):
    resolution = occupancy_grid_msg.info.resolution
    width = occupancy_grid_msg.info.width
    height = occupancy_grid_msg.info.height
    origin = occupancy_grid_msg.info.origin

    data = occupancy_grid_msg.data

    oc_grid = np.array(data).reshape((height, width))

    grid = []    #empty gird (list)
    rows = height    #number of rows
    columns = width   #number of columns

    for i in range(rows):    #create grid where all nodes are free
        row_nodes = []    #empty row
        for j in range(columns):    #fill row
            node = Node(j, i)    #create node
            node.type = oc_grid[i, j]
            row_nodes.append(node)    #add node to row
        grid.append(row_nodes)    #add row to grid

    return grid

#########################################################

def add_buffer(grid, buffer, start, end):
    
    rows = len(grid)
    columns = len(grid[0])
    count = 0
    for i in range(rows):
        for j in range(columns):
            print(f'Adding buffer zone: {(count / (len(grid) * len(grid[0])))*100:.2f} %', end = '\r')
            count = count + 1
            if grid[i][j].type > 51:    #if it's an obstacle
                for di in range(-buffer, buffer + 1):
                    for dj in range(-buffer, buffer + 1):
                        ni, nj = i + di, j + dj
                        if ni >= 0 and nj >= 0 and nj < columns and ni < rows:    #check if the neighbor is in bounds and within the buffer distance
                            if grid[ni][nj] != start and grid[ni][nj] != end and grid[ni][nj].type != 100:    #avoid overwriting start, end, or 100
                                    grid[ni][nj].type = 51    #set psudo obstacle
                                    
    return grid

#########################################################

def add_walls(grid):
    for i in range(len(grid)):
        grid[0][i].type = 100
        grid[i][0].type = 100
        grid[len(grid)-1][i].type = 100
        grid[i][len(grid[0])-1].type = 100

    return grid


#########################################################

def free_se(grid, buffer, start, end):
    buffer = buffer*2
    rows = len(grid)
    columns = len(grid[0])
    for i in range(rows):
        for j in range(columns):
            if grid[i][j] == start or grid[i][j] == end:    #if start or end
                for di in range(-buffer, buffer + 1):
                    for dj in range(-buffer, buffer + 1):
                        ni, nj = i + di, j + dj
                        if ni >= 0 and nj >= 0 and nj < columns and ni < rows:    #check if the neighbor is in bounds and within the buffer distance
                            grid[ni][nj].type = 0    #free start and end

    return grid

#########################################################

def a_star(occupancy_grid_msg, start_coor, end_coor):

    grid = process_occupancy_grid(occupancy_grid_msg)
    #grid = add_walls(grid)
    start = grid[start_coor[1]][start_coor[0]]
    end = grid[end_coor[1]][end_coor[0]]
    
    came_from = {}    #dictionary where keys are node coordinates and values is previous node
    next_nodes = []    #list where nodes are added which are to be examined next
    examined_nodes = []    #list where nodes are added that which were examined

    start.start_dis = 0    #distance to start node
    start.end_dis = dis_curr_end(start, end)    #distance to end node

    next_nodes.append(start)    #first node to examine
    grid = add_buffer(grid, buffer, start, end)    #add buffer zone
    #grid = free_se(grid, buffer, start, end)
    
    count = 0
    
    while len(next_nodes) > 0:    #loop until all nodes are examined
        print(f'Finding path, nodes examined: {(count / (len(grid) * len(grid[0])))*100:.2f} %', end = '\r')
        count = count + 1

        curr = min_end_dis(next_nodes)    #set the current node to the node of next_nodes that is closest to end
        next_nodes.remove(curr)
        examined_nodes.append(curr)

        if curr.x == end.x and curr.y == end.y:    #ends loop if end is found
            path = find_path(grid, came_from, curr)
            return path

        nei_nodes = get_nei(grid, curr)    #get neighbor nodes of current node
        """
        # Retrieve the previous node from came_from to get prev_dir
        key = f"{curr.x} {curr.y}"
        if key in came_from:
            prev_node = came_from[key]
        else:
            if abs(start.x - end.x) > abs(start.y - end.y):
                prev_node = Node(start.x - 1, start.y)  #horizontal
            elif abs(start.y - end.y) > abs(start.x - end.x):
                prev_node = Node(start.x, start.y - 1)  #vertical
        """

        for i in range(len(nei_nodes)):
            
            nei_node = nei_nodes[i]    #neighbor node i

            if nei_node in examined_nodes or nei_node.type > 50:    #do not examine node that are already examined or occupied
                continue

            adj_node_1 = grid[curr.y][nei_node.x]
            adj_node_2 = grid[nei_node.y][curr.x]
            if adj_node_1.type > 50 and adj_node_2.type > 50:    #do not examine diagonal neighbor node if both adjacent neighbor nodes are occupied
                continue

            start_dis_nei = curr.start_dis + dis_curr_nei(curr, nei_node)    #distance from neighbor node to start
            if nei_node not in next_nodes:    #add nei_node to next_nodes
                next_nodes.append(nei_node)

            elif start_dis_nei > nei_node.start_dis:    #do not examine node that moves us back to start
                continue

            came_from[str(nei_node.x) + ' ' + str(nei_node.y)] = curr    #puts current node into came_from with key of neighbor node
            nei_node.start_dis = start_dis_nei    #sets start_dis of neighbor node
            nei_node.end_dis = nei_node.start_dis + dis_curr_end(nei_node, end)    #sets end_dis of neighbor node
            
    print("No path found!")

#########################################################

def plot(occupancy_grid_msg, start, end):

    grid = process_occupancy_grid(occupancy_grid_msg)
    #grid = add_walls(grid)
    start_node = grid[start[1]][start[0]]
    end_node = grid[end[1]][end[0]]
    grid = add_buffer(grid, buffer, start_node, end_node)    #add buffer zone
    #grid = free_se(grid, buffer, start_node, end_node)

    
    plt.figure(dpi=300)
    plt.title('Occupancy Grid')

    columns1 = occupancy_grid_msg.info.width
    rows1 = occupancy_grid_msg.info.height
            
    oc1_x, oc1_y, path1_x, path1_y, poc1_x, poc1_y = [[] for _ in range(6)]
            
    for i in range(rows1):
        for j in range(columns1):
            if grid[i][j].type == 100:
                oc1_x.append(grid[i][j].x)
                oc1_y.append(grid[i][j].y)
            elif grid[i][j].type == 51:
                poc1_x.append(grid[i][j].x)
                poc1_y.append(grid[i][j].y)
                
    plt.plot(oc1_y, oc1_x, 's', color='black', markersize=0.05)
    plt.plot(poc1_y, poc1_x, 's', color='grey', markersize=0.05)

    plt.axis('equal')
    
    plt.show()
    
    path1 = a_star(occupancy_grid_msg, start, end)
    
    if path1 is None:
    	return
    
    #plot grid

    plt.figure(dpi=300)
    plt.title('Occupancy Grid with Path')
        
    for i in range(len(path1)):
        path1_x.append(path1[i].x)
        path1_y.append(path1[i].y)

    for i in range(rows1):
        for j in range(columns1):
            if grid[i][j].type == 51:
                poc1_x.append(grid[i][j].x)
                poc1_y.append(grid[i][j].y)
                
    plt.plot(path1_y, path1_x, color='red', linewidth=1, marker='o', markersize=0.01)
    plt.plot(oc1_y, oc1_x, 's', color='black', markersize=0.05)
    plt.plot(poc1_y, poc1_x, 's', color='grey', markersize=0.05)

    plt.axis('equal')
    
    plt.show()
