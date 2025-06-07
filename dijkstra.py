import heapq
from utils.workspace import create_random_workspace, grid_to_graph
from utils.plotting import plot_workspace

def dijkstra(graph, start, goal):
    """
    Perform Dijkstra's shortest-path algorithm on a weighted graph.

    Parameters:
    - graph: dict where keys are nodes (tuples) and values are lists of (neighbor, weight)
    - start: starting node (tuple)
    - goal: goal node (tuple)

    Returns:
    - path: list of nodes from start to goal (empty if no path)
    - cost: total path cost (float('inf') if no path)
    """
    pq = [(0, start)]
    costs = {start: 0}
    parent = {start: None}

    while pq:
        current_cost, current = heapq.heappop(pq)

        if current == goal:
            break

        # If we've already found a better path, skip
        if current_cost > costs.get(current, float('inf')):
            continue

        for neighbor, weight in graph.get(current, []):
            new_cost = current_cost + weight
            if new_cost < costs.get(neighbor, float('inf')):
                costs[neighbor] = new_cost
                parent[neighbor] = current
                heapq.heappush(pq, (new_cost, neighbor))

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()

    if not path or path[0] != start:
        return [], float('inf')
    return path, costs[goal]


if __name__ == "__main__":
    width, height = 100, 100
    seed = 10
    n_obs = 10
    
    # Random map/grid initialization
    obstacles, start, goal = create_random_workspace(width=width, height=height, n_obstacles=n_obs, obstacle_radius_range=(2, 10), seed=seed)

    graph = grid_to_graph(obstacles=obstacles, width=width, height=height, resolution=1, connectivity=4)
    
    path, cost = dijkstra(graph, start, goal)

    if path:
        print(f"Found path of length {len(path)} with cost {cost:.1f}")
    else:
        print("No path found!")

    plot_workspace(obstacles, start, goal, path=path, tree_edges=None, title=f"Dijkstra on Grid (cost={cost:.1f})")