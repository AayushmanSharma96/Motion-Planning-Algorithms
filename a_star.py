import heapq
from utils.workspace import createRandomGrid, grid_to_graph
from utils.plotting import plot_workspace

def manhDist(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(graph, start, goal):
    """
    Perform A* search on a weighted graph.

    Parameters:
    - graph: dict { node: [(neighbor_node, cost), ...], … }
    - start: tuple
    - goal:  tuple

    Returns:
    - path: list of nodes from start to goal (empty if none)
    - cost: total path cost (float('inf') if no path)
    """
    open_set = [(manhDist(start, goal), 0, start)]
    g_score = {start: 0}
    parent  = {start: None}

    while open_set:
        f_current, cost_current, current = heapq.heappop(open_set)
        if current == goal:
            break

        # Explore neighbors
        for neighbor, weight in graph.get(current, []):
            tentative_g = cost_current + weight
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                parent[neighbor] = current
                f_score = tentative_g + manhDist(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()

    if not path or path[0] != start:
        return [], float('inf')
    return path, g_score[goal]

if __name__ == "__main__":
    width, height = 100, 100
    seed = 52
    n_obs = 20
    
    # Random map/grid initialization
    obstacles, start, goal = createRandomGrid(width=width, height=height, n_obstacles=n_obs, obstacle_radius_range=(7, 10), seed=seed)
    graph = grid_to_graph(obstacles=obstacles, width=width, height=height, resolution=1, connectivity=8)

    path, cost = a_star(graph, start, goal)
    
    if path:
        print(f"A* found path of length {len(path)} with cost {cost:.1f}")
    else:
        print("A* did not find any path!")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=None, title=f"A* on Grid (cost={cost:.1f})")