from utils.workspace import create_random_workspace, grid_to_graph
from utils.plotting import plot_workspace

def depth_first_search(graph, start, goal):
    """
    Perform Depth First Search (DFS) on a graph.

    Parameters:
    - graph: dict {node: [neighbor nodes]}
    - start: start node
    - goal: goal node

    Returns:
    - path: list of nodes from start to goal (if found), else empty list
    """
    stack = [start]
    visited = set()
    parent = {start: None}

    while stack:
        current = stack.pop()

        if current == goal:
            break

        if current not in visited:
            visited.add(current)

            for neighbor,_ in graph.get(current, []):
                if neighbor not in visited:
                    parent[neighbor] = current
                    stack.append(neighbor)

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
    n_obs = 20
    
    # Random map/grid initialization
    obstacles, start, goal = create_random_workspace(width=width, height=height, n_obstacles=n_obs, obstacle_radius_range=(2, 10), seed=seed)

    graph = grid_to_graph(obstacles=obstacles, width=width, height=height, resolution=1, connectivity=4)

    path = depth_first_search(graph, start, goal)
    
    if path:
        print(f"DFS found path of length {len(path)}")
    else:
        print("DFS did not find any path!")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=None, title="Depth-First Search on Grid")

