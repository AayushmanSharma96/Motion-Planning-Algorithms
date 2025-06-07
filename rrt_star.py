import random
import math
from utils.workspace import createRandomGrid
from utils.plotting import plot_workspace

def checkCollision(p1, p2, obstacles, step_size=1.0):
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    steps = max(int(dist / step_size), 1)
    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        for (ox, oy, r) in obstacles:
            if (x - ox)**2 + (y - oy)**2 <= r**2:
                return False
    return True

def get_nearest(nodes, point):
    return min(nodes, key=lambda n: math.hypot(n[0]-point[0], n[1]-point[1]))

def get_nearby(nodes, point, radius):
    return [n for n in nodes if math.hypot(n[0]-point[0], n[1]-point[1]) <= radius]

def steer(from_pt, to_pt, step_size):
    theta = math.atan2(to_pt[1]-from_pt[1], to_pt[0]-from_pt[0])
    return (from_pt[0] + step_size*math.cos(theta),
            from_pt[1] + step_size*math.sin(theta))

def rrt_star(obstacles, start, goal, width, height, step_size=1.0, max_iter=100000, goal_sample_rate=0.05, neighbor_radius=15.0):
    """
    RRT* planner.

    Returns:
      path: list of waypoints from start to goal (empty if fail)
      tree_edges: list of ((x1,y1),(x2,y2)) for visualization
    """
    nodes     = [start]
    parent    = {start: None}
    cost      = {start: 0.0}
    tree_edges = []

    for _ in range(max_iter):
        # Sample
        rnd = goal if random.random() < goal_sample_rate else (random.uniform(0,width),
                                                               random.uniform(0,height))
        # Find nearest nodes
        nearest = get_nearest(nodes, rnd)
        new_pt  = steer(nearest, rnd, step_size)

        # Collision check
        if not (0 <= new_pt[0] <= width and 0 <= new_pt[1] <= height):
            continue
        if not checkCollision(nearest, new_pt, obstacles, step_size):
            continue

   
        near = get_nearby(nodes, new_pt, neighbor_radius)
        
        # Updating parent
        best_parent = nearest
        best_cost   = cost[nearest] + math.hypot(nearest[0]-new_pt[0], nearest[1]-new_pt[1])
        for n in near:
            c = cost[n] + math.hypot(n[0]-new_pt[0], n[1]-new_pt[1])
            if c < best_cost and checkCollision(n, new_pt, obstacles, step_size):
                best_parent, best_cost = n, c

        # Add new node
        nodes.append(new_pt)
        parent[new_pt] = best_parent
        cost[new_pt]   = best_cost
        tree_edges.append((best_parent, new_pt))

        for n in near:
            edge_cost = math.hypot(n[0]-new_pt[0], n[1]-new_pt[1])
            new_cost  = cost[new_pt] + edge_cost
            if new_cost < cost[n] and checkCollision(new_pt, n, obstacles, step_size):
                parent[n] = new_pt
                cost[n]   = new_cost
                tree_edges.append((new_pt, n))

        if math.hypot(new_pt[0]-goal[0], new_pt[1]-goal[1]) <= step_size:
            parent[goal] = new_pt
            cost[goal]   = cost[new_pt] + math.hypot(new_pt[0]-goal[0], new_pt[1]-goal[1])
            tree_edges.append((new_pt, goal))
            nodes.append(goal)
            break

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()

    if not path or path[0] != start:
        return [], tree_edges
    return path, tree_edges

if __name__ == "__main__":
    width, height = 100, 100
    seed = 10
    n_obs = 20
    
    # Random map/grid initialization
    obstacles, start, goal = createRandomGrid(width=width, height=height, n_obstacles=n_obs, obstacle_radius_range=(7, 10), seed=seed)
    path, tree_edges = rrt_star(obstacles, start, goal, width, height, step_size=1.0, max_iter=100000, goal_sample_rate=0.02, neighbor_radius=10.0) 

    if path:
        print(f"RRT* found path of length {len(path)}; cost={round(sum(math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]) for i in range(1,len(path))),2)}")
    else:
        print("RRT* failed to find a path")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=tree_edges, title=f"RRT*")
