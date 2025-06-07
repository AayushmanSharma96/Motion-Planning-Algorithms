import numpy as np

def create_random_workspace(width=100,
                            height=100,
                            n_obstacles=10,
                            obstacle_radius_range=(5, 15),
                            seed=42):
    """
    Create a 2D workspace with random circular obstacles.
    Returns:
      obstacles: list of (x_center, y_center, radius)
      start: (x, y)
      goal: (x, y)
    """
    obstacles = []
    np.random.seed(seed)

    for _ in range(n_obstacles):
        x = np.random.uniform(0, width)
        y = np.random.uniform(0, height)
        r = np.random.uniform(*obstacle_radius_range)
        obstacles.append((x, y, r))

    start = (10, 10)
    goal  = (90, 90)
    return obstacles, start, goal


def point_in_obstacle(pt, obstacles):
    x, y = pt
    for (ox, oy, r) in obstacles:
        if (x - ox)**2 + (y - oy)**2 <= r**2:
            return True
    return False


def grid_to_graph(obstacles,
                  width,
                  height,
                  resolution=1,
                  connectivity=4):
    """
    Convert a grid (width x height) into a graph of collision-free nodes.

    Parameters:
    - obstacles: list of (x_center, y_center, radius)
    - width, height: grid size
    - resolution: size of each cell (assumes integer grid if =1)
    - connectivity: 4 or 8 for neighbor connectivity

    Returns:
    - graph: dict mapping (i,j) -> [(neighbor_coord, cost), ...]
    """
    # define neighbor offsets
    if connectivity == 4:
        deltas = [(1,0),(-1,0),(0,1),(0,-1)]
    elif connectivity == 8:
        deltas = [(1,0),(-1,0),(0,1),(0,-1),
                  (1,1),(1,-1),(-1,1),(-1,-1)]
    else:
        raise ValueError("connectivity must be 4 or 8")

    graph = {}
    # iterate over grid cells
    for i in range(0, width, resolution):
        for j in range(0, height, resolution):
            pt = (i, j)
            # skip if center lies inside any obstacle
            if point_in_obstacle(pt, obstacles):
                continue

            neighbors = []
            for dx, dy in deltas:
                ni, nj = i + dx*resolution, j + dy*resolution
                if 0 <= ni < width and 0 <= nj < height:
                    nbr = (ni, nj)
                    if not point_in_obstacle(nbr, obstacles):
                        # cost = Euclidean distance
                        cost = np.hypot(dx*resolution, dy*resolution)
                        neighbors.append((nbr, cost))
            graph[pt] = neighbors

    return graph
