import math
import numpy as np
import matplotlib.pyplot as plt

#########################################################

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.type = 'free'    #types: free, oc (= occupied), path, start, end
        self.start_dis = float('inf')    #distance to start node
        self.end_dis = float('inf')    #distance to end node

#########################################################

#create grid
grid = []    #empty gird (list)
rows = 100    #number of rows
columns = 100   #number of columns

for i in range(rows):    #create grid where all nodes are free
    row_nodes = []    #empty row
    for j in range(columns):    #fill row
        node = Node(j, i)    #create node
        row_nodes.append(node)    #add node to row
    grid.append(row_nodes)    #add row to grid

#########################################################

#create obstacles
x_pos = [np.arange(0, 95, 1), np.arange(5, 100, 1)]
y_pos = np.array([33, 66])

for i in range(len(y_pos)):
    for j in range(len(x_pos[i])):
        x = x_pos[i][j]
        y = y_pos[i]
        grid[y][x].type = 'oc'    #occupied

#########################################################

#set start node
start_coor = np.array([0, 0])    #[x, y]
grid[start_coor[1]][start_coor[0]].type = 'start'    #set start
start = grid[start_coor[1]][start_coor[0]]

#set end node
end_coor = np.array([99, 99])    #[x, y]
grid[end_coor[1]][end_coor[0]].type = 'end'    #set end
end = grid[end_coor[1]][end_coor[0]]

#########################################################

def dis_curr_end(curr, end):    #minimal distance from current node to end node
    
    x_dis = abs(end.x - curr.x)
    y_dis = abs(end.y - curr.y)
    
    diag = min(x_dis, y_dis)    #minimal diagonal steps needed to reach end
    strai = x_dis + y_dis - 2 * diag    #minimal straight steps needed to reach end
    
    dis = math.sqrt(2) * diag + strai
    
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
    
    direc = np.array([[1, 0], [1, 1], [0, 1], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]])    #possible directions (vertical, horizontal and diagonal)
    
    nei_nodes = []    #list where neighbor nodes are added
    
    for j in range(8):
        
        direc_j = direc[j,:]
        
        nei_x = curr.x + direc_j[0]
        nei_y = curr.y + direc_j[1]

        if nei_x >= 0 and nei_y >= 0 and nei_x < columns and nei_y < rows:    #adds neighbor for every direction if in domain
            nei_nodes.append(grid[nei_y][nei_x])

    return nei_nodes

#########################################################

def dis_curr_nei(curr, nei_node):
    dis = math.sqrt(((curr.x - nei_node.x)**2) + ((curr.y - nei_node.y)**2))    #distance between two nodes
    return dis

#########################################################

def find_path(grid, came_from, curr):    #reconstructs the optimal path from came_from
    path = []
    key = str(curr.x) + ' ' + str(curr.y)
    while key in came_from:
        
        path.insert(0, curr)
        curr = came_from[key]
        key = str(curr.x) + ' ' + str(curr.y)
        curr.type = 'path'
        
    return path

#########################################################

def a_star(grid, start, end):
    
    next_nodes = []    #list where nodes are added which are to be examined next
    examined_nodes = []    #list where nodes are added that which were examined
    came_from = {}    #dictionary where keys are node coordinates and values is previous node

    start.start_dis = 0    #distance to start node
    start.end_dis = dis_curr_end(start, end)    #distance to end node

    next_nodes.append(start)    #first node to examine

    while len(next_nodes) > 0:    #loop until all nodes are examined

        curr = min_end_dis(next_nodes)    #set the current node to the node of next_nodes that is closest to end
        next_nodes.remove(curr)
        examined_nodes.append(curr)

        if curr == end:    #ends loop if end is found
            path = find_path(grid, came_from, curr)
            return path

        nei_nodes = get_nei(grid, curr)    #get neighbor nodes of current node
        
        for i in range(len(nei_nodes)):
            
            nei_node = nei_nodes[i]    #neighbor node i

            if nei_node in examined_nodes or nei_node.type == 'oc':    #do not examine node that are already examined or occupied
                continue

            adj_node_1 = grid[curr.y][nei_node.x]
            adj_node_2 = grid[nei_node.y][curr.x]
            if adj_node_1.type == 'oc' and adj_node_2.type == 'oc':    #do not examine diagonal neighbor node if both adjacent neighbor nodes are occupied
                continue

            start_dis_nei = curr.start_dis + dis_curr_nei(curr, nei_node)    #distance from neighbor node to start
            if nei_node not in next_nodes:    #add nei_node to next_nodes
                next_nodes.append(nei_node)
            elif start_dis_nei > nei_node.start_dis:    #do not examine node that moves us back to start
                continue

            came_from[str(nei_node.x) + ' ' + str(nei_node.y)] = curr    #puts current node into came_from with key of neighbor node
            nei_node.start_dis = start_dis_nei    #sets start_dis of neighbor node
            nei_node.end_dis = nei_node.start_dis + dis_curr_end(nei_node, end)    #sets end_dis of neighbor node

#########################################################

#testing
path = a_star(grid, start, end)

#########################################################

#display grid
plt.figure(figsize=(4.8, 4.8), dpi=300)
plt.axis((-1, rows, -1, columns))

#sort nodes by type
oc_x, oc_y, free_x, free_y, path_x, path_y, se_x, se_y = [[] for _ in range(8)]
for i in range(rows):
    for j in range(columns):
        if grid[j][i].type == 'oc':
            oc_x.append(grid[j][i].x)
            oc_y.append(grid[j][i].y)
        elif grid[j][i].type == 'free':
            free_x.append(grid[j][i].x)
            free_y.append(grid[j][i].y)
        elif grid[j][i].type == 'path':
            path_x.append(grid[j][i].x)
            path_y.append(grid[j][i].y)
        elif grid[j][i].type == 'start' or grid[j][i].type == 'end':
            se_x.append(grid[j][i].x)
            se_y.append(grid[j][i].y)

#display nodes by type
plt.plot(oc_x, oc_y, 'o', color='black', markersize=1)
plt.plot(free_x, free_y, 'o', color='green', markersize=1)
plt.plot(path_x, path_y, 'o', color='blue', markersize=1)
plt.plot(se_x, se_y, 'o', color='red', markersize=1)
plt.show()