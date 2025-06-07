import random
import math
#from utils.workspace import createRandomGrid
from utils.workspace import createCustomGrid
from utils.plotting import plot_workspace

def collision_free(p1, p2, obstacles, step_size=1.0):
    """
    Check that the straight-line path from p1 to p2 does not intersect any circular obstacle.
    """
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

def rrt(obstacles, start, goal,
        width, height,
        step_size=1.0,
        max_iter=100000,
        goal_sample_rate=0.05):
    """
    Rapidly-exploring Random Tree (RRT)

    Returns:
    - path: list of waypoints from start to goal (empty if fail)
    - tree_edges: list of ((x1,y1),(x2,y2)) edges for visualization
    """
    nodes = [start]
    parent = {start: None}
    tree_edges = []

    for _ in range(max_iter):
        # Sample random point (occasionally bias toward goal)
        if random.random() < goal_sample_rate:
            rnd = goal
        else:
            rnd = (random.uniform(0, width), random.uniform(0, height))

        # Find nearest existing node
        nearest = min(nodes,
                      key=lambda n: math.hypot(n[0] - rnd[0], n[1] - rnd[1]))

        # Steer toward rnd by step_size
        theta = math.atan2(rnd[1] - nearest[1], rnd[0] - nearest[0])
        new_pt = (nearest[0] + step_size * math.cos(theta),
                  nearest[1] + step_size * math.sin(theta))

        # Skip if out of bounds or in collision
        if not (0 <= new_pt[0] <= width and 0 <= new_pt[1] <= height):
            continue
        if not collision_free(nearest, new_pt, obstacles, step_size=step_size):
            continue

        # Add to tree
        nodes.append(new_pt)
        parent[new_pt] = nearest
        tree_edges.append((nearest, new_pt))

        # Check if we can connect directly to goal
        if math.hypot(new_pt[0] - goal[0], new_pt[1] - goal[1]) <= step_size:
            parent[goal] = new_pt
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

    # Random map/grid initialization
    width, height = 100, 100
    
    obstacles, start, goal = createCustomGrid(width=width, height=height, x_obs=[50, 50], y_obs=[25, 75], rad = [23, 23])

    path, tree_edges = rrt(obstacles, start, goal, width, height, step_size=0.5, max_iter=10000, goal_sample_rate=0.05)

    if path:
        print(f"RRT found a path of {len(path)} nodes")
    else:
        print("RRT failed to find a path")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=tree_edges, title="RRT")