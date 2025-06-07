from collections import deque
from utils.workspace import createRandomGrid, grid_to_graph
from utils.plotting import plot_workspace


def breadth_first_search(graph, start, goal):
    """
    Perform Breadth First Search (BFS) on a graph.

    Parameters:
    - graph: dict {node: [neighbor nodes]}
    - start: start node
    - goal: goal node

    Returns:
    - path: list of nodes from start to goal (if found), else empty list
    """
    queue = deque([start])
    visited = set()
    parent = {start: None}

    while queue:
        current = queue.popleft()

        if current == goal:
            break

        for neighbor,_ in graph.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)

    path.reverse()

    if path[0] == start:
        return path
    else:
        return []  # no path found

if __name__ == "__main__":
    width, height = 100, 100
    seed = 10
    n_obs = 10
    
    # Random map/grid initialization
    obstacles, start, goal = createRandomGrid(width=width, height=height, n_obstacles=n_obs, obstacle_radius_range=(2, 10), seed=seed)
    graph = grid_to_graph(obstacles=obstacles, width=width, height=height, resolution=1, connectivity=4)

    path = breadth_first_search(graph, start, goal)
    
    if path:
        print(f"BFS found path of length {len(path)}")
    else:
        print("BFS did not find any path!")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=None, title="Breadth-First Search on Grid")

