from pathlib import Path
import matplotlib.pyplot as plt

def load_env(path):
    path = Path(path)
    with path.open() as f:
        header = f.readline()
        m = int(f.readline().strip())
        obstacles = []
        for _ in range(m):
            k = int(f.readline().strip())
            poly = []
            for _ in range(k):
                x, y = map(float, f.readline().split())
                poly.append((x, y))
            obstacles.append(poly)
        sx, sy = map(float, f.readline().split())
        gx, gy = map(float, f.readline().split())
    return obstacles, (sx, sy), (gx, gy)

def load_graph(path):
    path = Path(path)
    if not path.exists():
        return {}, []
    with path.open() as f:
        N = int(f.readline().strip())
        nodes = {}
        for _ in range(N):
            nid, x, y = f.readline().split()
            nodes[int(nid)] = (float(x), float(y))
        M = int(f.readline().strip())
        edges = []
        for _ in range(M):
            u, v, w = f.readline().split()
            edges.append((int(u), int(v), float(w)))
    return nodes, edges

def load_path(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open() as f:
        n = int(f.readline().strip())
        pts = [tuple(map(float, f.readline().split())) for _ in range(n)]
    return pts

def visualize(stage, env_path, graph_path=None, path_path=None):
    obstacles, start, goal = load_env(env_path)
    nodes, edges = ({} ,[])
    path = []

    if stage >= 2 and graph_path is not None:
        nodes, edges = load_graph(graph_path)

    if stage >= 3 and path_path is not None:
        path = load_path(path_path)

    fig, ax = plt.subplots()

    # 1) obstacles
    for poly in obstacles:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.fill(xs, ys, alpha=0.3)

    # 2) roadmap (if stage >= 2)
    if stage >= 2 and nodes:
        for u, v, w in edges:
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            ax.plot([x1, x2], [y1, y2], linewidth=0.3)
        xs = [p[0] for p in nodes.values()]
        ys = [p[1] for p in nodes.values()]
        ax.scatter(xs, ys, s=5)

    # 3) path (if stage >= 3)
    if stage >= 3 and path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, linewidth=2)

    # start / goal
    ax.scatter([start[0]], [start[1]], s=50, marker='o', label="start")
    ax.scatter([goal[0]], [goal[1]], s=50, marker='*', label="goal")

    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect('equal', 'box')
    ax.set_title(f"Stage {stage}")
    ax.legend()
    plt.show()
