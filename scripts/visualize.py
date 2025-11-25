from pathlib import Path
from datetime import datetime
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from scripts.utils import get_stage
import os

# ---------------------------- Utilities ----------------------------

def load_env(path):
    path = Path(path)
    with path.open() as f:
        f.readline()  # skip seed line
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


def _draw_scene(ax, obstacles, start, goal,
                nodes=None, edges=None, path=None):
    # Obstacles
    for poly in obstacles:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.fill(xs, ys, alpha=0.3, edgecolor="black")

    # Roadmap edges
    if edges:
        for e in edges:
            u, v = e[0], e[1]
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            ax.plot([x1, x2], [y1, y2], linewidth=0.3)

    # Roadmap nodes
    if nodes:
        xs = [p[0] for p in nodes.values()]
        ys = [p[1] for p in nodes.values()]
        ax.scatter(xs, ys, s=5)

    # Path
    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, linewidth=3, color="red")

    # Start / Goal
    ax.scatter([start[0]], [start[1]], s=60, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], s=60, marker="*", label="goal")

    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect("equal", "box")



# ---------------------------- Helper Functions ----------------------------

def _save_graph_png(root_dir, roadmap, obstacles, start, goal,
                    nodes, edges):
    fig, ax = plt.subplots()
    _draw_scene(ax, obstacles, start, goal, nodes, edges)
    ax.set_title(f"Roadmap: {roadmap}")
    ax.legend(loc="upper left")
    outfile = root_dir / f"map_{roadmap}.png"
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[visualize] Saved {outfile}")


def _save_path_png(root_dir, roadmap, search,
                   obstacles, start, goal, nodes, edges, path):
    fig, ax = plt.subplots()
    _draw_scene(ax, obstacles, start, goal, nodes, edges, path)
    ax.set_title(f"Path: {roadmap} + {search}")
    ax.legend(loc="upper left")
    outfile = root_dir / f"map_{roadmap}_{search}.png"
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[visualize] Saved {outfile}")


def _save_graph_gif(root_dir, roadmap, obstacles, start, goal,
                    nodes_all, edges_all, graph_path):

    steps_path = Path(str(graph_path) + ".steps")
    if not steps_path.exists():
        print("[visualize] No steps file → graph GIF skipped")
        return

    frames = []
    nodes_present = set([0, 1])
    edges_present = []

    with steps_path.open() as f:
        in_step = False
        for line in f:
            line = line.strip()

            if line == "STEP":
                in_step = True
                continue

            if line == "END":
                if in_step:
                    fig_s, ax_s = plt.subplots()
                    nodes_sub = {nid: nodes_all[nid] for nid in nodes_present if nid in nodes_all}

                    _draw_scene(ax_s, obstacles, start, goal,
                                nodes_sub, edges_present)
                    fig_s.canvas.draw()
                    w, h = fig_s.canvas.get_width_height()
                    buf = fig_s.canvas.buffer_rgba()
                    img = Image.frombuffer("RGBA", (w, h), buf, "raw", "RGBA", 0, 1)
                    frames.append(img)
                    plt.close(fig_s)

                in_step = False
                continue

            if line.startswith("NODE"):
                nid = int(line.split()[1])
                nodes_present.add(nid)

            elif line.startswith("EDGE"):
                _, u, v = line.split()
                edges_present.append((int(u), int(v)))

    if frames:
        outfile = root_dir / f"map_{roadmap}.gif"
        frames[0].save(outfile, save_all=True,
                       append_images=frames[1:], duration=80, loop=0)
        print(f"[visualize] Saved {outfile}")


def _save_path_gif(root_dir, roadmap, search,
                   obstacles, start, goal, path):

    if not path:
        print("[visualize] Path empty → path GIF skipped")
        return

    frames = []
    partial = []

    for point in path:
        partial.append(point)

        fig_s, ax_s = plt.subplots()
        _draw_scene(ax_s, obstacles, start, goal, path=partial)
        fig_s.canvas.draw()
        w, h = fig_s.canvas.get_width_height()
        buf = fig_s.canvas.buffer_rgba()
        img = Image.frombuffer("RGBA", (w, h), buf, "raw", "RGBA", 0, 1)
        frames.append(img)
        plt.close(fig_s)

    outfile = root_dir / f"map_{roadmap}_{search}.gif"
    frames[0].save(outfile, save_all=True,
                   append_images=frames[1:], duration=120, loop=0)
    print(f"[visualize] Saved {outfile}")


# ---------------------------- Main Visualizer ----------------------------
ROOT_RESULT = "/home/changmin/PyProject/planning-example/results"
def visualize(args):
    stage = get_stage(args)
    
    roadmap = args.roadmap
    search = args.search_method

    env_path = os.path.join(ROOT_RESULT, "env.txt")
    graph_path = os.path.join(ROOT_RESULT, "graph.txt")
    path_path = os.path.join(ROOT_RESULT, "path.txt")
    save = args.save

    # -----------------------------
    # 1) output folder
    # -----------------------------
    timestamp = datetime.now().strftime("%y%m%d-%H%M")
    root_dir = Path(env_path).parent.parent / "results" / f"re-{timestamp}"
    root_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------
    # Load data
    # -----------------------------
    obstacles, start, goal = load_env(env_path)

    nodes_all, edges_all = {}, []
    if stage >= 2:
        nodes_all, edges_all = load_graph(graph_path)

    path = []
    if stage >= 3:
        path = load_path(path_path)

    # -----------------------------
    # Stage 1 → simple PNG
    # -----------------------------
    if stage == 1:
        base = root_dir / "map"
        fig, ax = plt.subplots()
        _draw_scene(ax, obstacles, start, goal)
        ax.legend(loc="upper left")
        if save:
            fig.savefig(base.with_suffix(".png"), dpi=150)
        plt.close(fig)
        print(f"[visualize] Saved PNG: {base}.png")
        return

    # -----------------------------
    # Stage 2 → PNG + graph GIF
    # -----------------------------
    if stage == 2:
        _save_graph_png(root_dir, roadmap, obstacles, start, goal, nodes_all, edges_all)
        _save_graph_gif(root_dir, roadmap, obstacles, start, goal,
                        nodes_all, edges_all, graph_path)
        return

    # -----------------------------
    # Stage 3 → PNG + (graph GIF + path GIF)
    # -----------------------------
    if stage == 3:
        # 1) stage2 GIF
        _save_graph_png(root_dir, roadmap, obstacles, start, goal, nodes_all, edges_all)
        _save_graph_gif(root_dir, roadmap, obstacles, start, goal,
                        nodes_all, edges_all, graph_path)

        # 2) stage3 GIF - path search
        _save_path_png(root_dir, roadmap, search,
                       obstacles, start, goal, nodes_all, edges_all, path)
        _save_path_gif(root_dir, roadmap, search,
                       obstacles, start, goal, path)
        return