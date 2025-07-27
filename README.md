# Motion-Planning-Algorithms 🚗🤖

A repository containing **clean, educational implementations** of classic motion planning algorithms in Python, with visualization examples. Ideal for learning, teaching, or bootstrapping robotics and simulation projects.

---

## ✨ Key Algorithms
| Algorithm         | Description                                                                    |
|-------------------|--------------------------------------------------------------------------------|
| Depth-First Search (DFS) | Grid-based depth-first search for path planning.
| A\*               | Grid-based, heuristic search for optimal path planning.                       |
| Dijkstra          | Uniform-cost search for shortest paths in weighted graphs.                    |
| Rapidly-Exploring Random Tree (RRT)    | Sampling-based planner for high-dimensional spaces.        |
| RRT\*             | Asymptotically optimal variant of RRT.                                        |
| Probabilistic Roadmap (PRM)            | Global roadmap-based approach for multi-query planning. |

---

## 📦 Installation
```bash
# Clone the repo
git clone https://github.com/AayushmanSharma96/Motion-Planning-Algorithms.git
cd Motion-Planning-Algorithms

# (Optional) create a virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt  # e.g., networkx, matplotlib, numpy
```

---

## 🚀 Usage

Each algorithm script can be run directly. For example:

```bash
# Run A* on a sample grid map
python algos/astar.py --Map examples/maps/simple_grid.txt --visualize

# Run RRT* in a 2D environment
python algos/rrt_star.py --start 0 0 --goal 10 10 --iterations 500 --visualize
```

Check the `examples/` folder for ready-to-run scripts and custom maps.

---

## 🗂️ Repo Layout
```
Motion-Planning-Algorithms/
├─ algos/
│   ├─ astar.py
│   ├─ dijkstra.py
│   ├─ rrt.py
│   ├─ rrt_star.py
│   └─ prm.py
├─ examples/
│   ├─ simple_astar_example.py
│   ├─ rrt_visualization.py
│   └─ maps/
│       └─ simple_grid.txt
├─ requirements.txt
└─ README.md
```

---

## 📈 Visualization

- **Graph Search**  
<p align="center">
  <img src="plots/dfs_plot.png" alt="DFS Path Example" width="60%" />
</p>

<p align="center">
  <img src="plots/dijkstra_plot.png" alt="Dijkstra Example" width="60%" />
</p>

<p align="center">
  <img src="plots/a_star_plot.png" alt="A* Path Example" width="60%" />
</p>

- **Sampling-Based**  
<p align="center">
  <img src="plots/rrt_plot.png" alt="RRT Path Example" width="60%" />
</p>
<p align="center">
  <img src="plots/rrt_star_plot.png" alt="RRT* Path Example" width="60%" />
</p>

---



