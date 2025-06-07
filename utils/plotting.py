import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_workspace(obstacles, start, goal, path=None, tree_edges=None, title="Workspace"):
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Plot obstacles
    for (x, y, r) in obstacles:
        circle = patches.Circle((x, y), r, color='gray', alpha=0.5)
        ax.add_patch(circle)

    # Plot start and goal
    ax.plot(start[0], start[1], "go", markersize=10, label="Start")
    ax.plot(goal[0], goal[1], "ro", markersize=10, label="Goal")

    # Plot tree edges if given (for RRT / RRT*)
    if tree_edges is not None:
        for (p1, p2) in tree_edges:
            x_vals = [p1[0], p2[0]]
            y_vals = [p1[1], p2[1]]
            ax.plot(x_vals, y_vals, color="blue", linewidth=0.5)

    # Plot path if given
    if path is not None:
        x_vals = [p[0] for p in path]
        y_vals = [p[1] for p in path]
        ax.plot(x_vals, y_vals, color="orange", linewidth=2, label="Path")

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    plt.show()
