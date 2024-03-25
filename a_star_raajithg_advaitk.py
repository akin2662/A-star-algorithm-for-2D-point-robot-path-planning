import numpy as np
import cv2
import math
import time
import heapq as hq

#Define map boundaries
map_xbounds, map_ybounds = 1200, 500

#Define the Obstacle Equations
def all_obstacles(clearance, radius):

    obs = {
        "Rectangle1": lambda x, y: 0 <= y <= 400 and 100 <= x <= 175,
        "Rectangle2": lambda x, y: 100 <= y <= 500 and 275 <= x <= 350,
        "Hexagon": lambda x, y: (150/2) * abs(x-650)/150 + 100 <= y <= 400 - (150/2) * abs(x-650)/150 and 520 <= x <= 780,
        "Rectangle3": lambda x, y: 50 <= y <= 125 and 900 <= x <= 1100,
        "Rectangle4": lambda x, y: 375 <= y <= 450 and 900 <= x <= 1100,
        "Rectangle5": lambda x, y: 50 <= y <= 450 and 1020 <= x <= 1100,
    }

    t_clearance = clearance + radius
    pixels_bounds = np.full((map_ybounds, map_xbounds, 3), 255, dtype=np.uint8)

    for i in range(map_ybounds):
        for j in range(map_xbounds):
            is_obstacle = any(eqn(j, i) for eqn in obs.values())
            if is_obstacle:
                pixels_bounds[i, j] = [0, 0, 255]  # obstacle
            else:
                is_clearance = any(
                    eqn(x, y)
                    for eqn in obs.values()
                    for y in range(i - t_clearance, i + t_clearance + 1)
                    for x in range(j - t_clearance, j + t_clearance + 1)
                    if (x - j)**2 + (y - i)**2 <= t_clearance**2
                )
                if i < t_clearance or i >= map_ybounds - t_clearance or j < t_clearance or j >= map_xbounds - t_clearance:
                    pixels_bounds[i, j] = [160, 0, 0]  # boundary
                elif is_clearance:
                    pixels_bounds[i, j] = [0, 255, 0]  # clearance
                else:
                    pixels_bounds[i, j] = [255, 255, 255]  # free space
    return pixels_bounds

# Define the move functions
def move_60_positive(p, L):
    x_move = np.ceil(p[0] - L * math.cos(math.radians(60 + p[2])))
    y_move = np.ceil(p[1] - L * math.sin(math.radians(60 + p[2])))
    o_move = (p[2] - 60) % 360
    return (x_move, y_move, o_move), 1

def move_30_positive(p, L):
    x_move = np.ceil(p[0] - L * math.cos(math.radians(30 + p[2])))
    y_move = np.ceil(p[1] - L * math.sin(math.radians(30 + p[2])))
    o_move = (p[2] - 30) % 360
    return (x_move, y_move, o_move), 1

def move_front(p, L):
    x_move = np.ceil(p[0] + L * math.cos(math.radians(p[2])))
    y_move = np.ceil(p[1] + L * math.sin(math.radians(p[2])))
    o_move = p[2]
    return (x_move, y_move, o_move), 1

def move_30_negative(p, L):
    x_move = np.ceil(p[0] + L * math.cos(math.radians(-30 + p[2])))
    y_move = np.ceil(p[1] + L * math.sin(math.radians(-30 + p[2])))
    o_move = (p[2] + 30) % 360
    return (x_move, y_move, o_move), 1

def move_60_negative(p, L):
    x_move = np.ceil(p[0] + L * math.cos(math.radians(-60 + p[2])))
    y_move = np.ceil(p[1] + L * math.sin(math.radians(-60 + p[2])))
    o_move = (p[2] + 60) % 360
    return (x_move, y_move, o_move), 1


# Define a function to check if current node is in range
def check_range(node):
    x, y, o = node
    y = map_ybounds - y - 1
    return 0 <= x < map_xbounds and 0 <= y < map_ybounds and (pixels[int(y), int(x)] == [255, 255, 255]).all() and o%30 == 0

def check_valid_node(node, visited):
    if not check_range(node):
        return False
    x, y, _ = node
    y = map_ybounds - y - 1
    if not (pixels[int(y), int(x)] == [255, 255, 255]).all():
        return False
    # To Check the node is within threshold distance from any visited nodes
    threshold_dist_x = 0.5
    threshold_dist_y = 0.5
    threshold_theta = math.radians(30)
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                neighbor_node = (x + i * threshold_dist_x, y + j * threshold_dist_y, k * threshold_theta)
                if neighbor_node in visited:
                    return False
    return True

# Define a function to check if current node is the goal node
def is_goal(current_node, goal_node):
    return np.sqrt((goal_node[0]-current_node[0])**2 + (goal_node[1]-current_node[1])**2) <= 1.5

# Define a function to calculate the euclidean distance
def euclidean_distance(node1, g_node):
    x1, y1, _ = node1
    x2, y2, _ = g_node
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, current_node = [goal_node], goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(start_node)
    return path[::-1]


# Define the A* algorithm
def a_star(start_node, goal_node, display_animation=True):
    threshold = 1.5 # threshold distance
    r = int(map_ybounds / threshold)  # number of rows
    c = int(map_xbounds / threshold)   # number of columns
    angles = int(360 / 30)           # number of angles
    visited_node_matrix = [[[False for _ in range(angles)] for _ in range(c)] for _ in range(r)]    # visited nodes matrix

    olist = []
    hq.heapify(olist)
    clist = set()
    c2c = {start_node: 0}
    c2g = {start_node: euclidean_distance(start_node, goal_node)}
    cost = {start_node: c2c[start_node] + c2g[start_node]}
    parent = {start_node: None}
    hq.heappush(olist,(cost[start_node], start_node))
    visited = set([start_node])
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output_video.mp4', fourcc, 15.0, (map_xbounds, map_ybounds))

    while olist:
        _, current_node = hq.heappop(olist)
        clist.add(current_node)
        x, y, theta = current_node
        visited_node_matrix[int(y / threshold)][int(x / threshold)][int(theta / 30)] = True   # Mark current node as visited
        out.write(pixels)
        if display_animation:
            cv2.imshow('Explored', pixels)
            cv2.waitKey(1)
        # Check if current node is the goal node
        if is_goal(current_node, goal_node):
            approx_goal_node = current_node  # Approximate goal node (within threshold distance)
            cost[goal_node] = cost[current_node]   # Cost of goal node
            path = backtrack_path(parent, start_node, approx_goal_node)  # Backtrack the path
            if display_animation:
                for node in path:
                    x, y, _ = node
                    cv2.circle(pixels, (int(x), map_ybounds - 1 - int(y)), 1, (0, 255, 0), thickness=-1)
                out.write(pixels)
                cv2.waitKey(0)
            print("Final Cost: ", cost[goal_node])
            out.release()
            cv2.destroyAllWindows()
            return path

        for move_func in [move_30_negative, move_60_negative, move_front, move_30_positive, move_60_positive]:    # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, L)
            if check_valid_node(new_node, visited):    # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(new_node[2] / 30)  # Get the index of the node in the 3D array
                if not visited_node_matrix[i][j][k]:  # Check if the node is in closed list
                    new_c2c = c2c[current_node] + move_cost
                    new_c2g = euclidean_distance(new_node, goal_node)
                    new_cost = new_c2c + new_c2g    # Update cost
                    if new_node not in c2c or new_c2c < c2c[new_node]:
                        c2c[new_node] = new_c2c   # Update cost to come
                        c2g[new_node] = new_c2g    # Update cost to go
                        cost[new_node] = new_cost   # Update cost
                        parent[new_node] = current_node  # Update parent
                        hq.heappush(olist,(new_cost,new_node))  # Add to open list
                        visited.add(new_node)   # Add to visited list

                        # Draw vector from current_node to new_node
                        cv2.line(pixels, (int(current_node[0]), map_ybounds - 1 - int(current_node[1])), (int(new_node[0]), map_ybounds - 1 - int(new_node[1])), (0, 0, 0), thickness=1)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break

    out.release()
    cv2.destroyAllWindows()
    return None

# Get valid start and goal nodes from user input
while True:
    print("NOTE:")
    print("1) Please give orientation values as {...-60,-30,0,30,60...}")
    print("2) The step size should be between 1 and 10")
    print("3) Please be patient, it may take some time to get the output\n")

    clearance = int(input("Clearance from the obstacles: "))
    radius = int(input("Radius of the robot: "))
    pixels = all_obstacles(clearance, radius)
    start_node = tuple(map(int, input("Enter the start node : ").split()))
    if not check_range(start_node):
        print("Please reenter a new node!")
        continue
    goal_node = tuple(map(int, input("Enter the goal node: ").split()))
    if not check_range(goal_node):
        print("Please reenter a new node!")
        continue
    L = int(input("Step size of the robot: "))
    if L < 1 or L > 10:
        print("Please reenter the step size as a value between 1 and 10")
        continue
    break

# Run A* algorithm
start_time = time.time()
path = a_star(start_node,goal_node)
if path is None:
    print("\nError: No path found.")
else:
    print("\nGoal Node Reached!\nShortest Path: ", path, "\n")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n")