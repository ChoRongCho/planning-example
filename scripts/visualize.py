from pathlib import Path
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Agg")

from PIL import Image


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


def _draw_scene(ax, obstacles, start, goal, nodes=None, edges=None, path=None):
    for poly in obstacles:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.fill(xs, ys, alpha=0.3, edgecolor="black")

    if nodes and edges:
        for e in edges:
            u, v = e[0], e[1]
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            ax.plot([x1, x2], [y1, y2], linewidth=0.3)
        xs = [p[0] for p in nodes.values()]
        ys = [p[1] for p in nodes.values()]
        ax.scatter(xs, ys, s=5)

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        ax.plot(px, py, linewidth=2)

    ax.scatter([start[0]], [start[1]], s=50, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], s=50, marker="*", label="goal")

    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect("equal", "box")


def visualize(stage, env_path, graph_path=None, path_path=None, save=True,
              roadmap=None, search=None):

    # -----------------------------
    # 1) 결과 저장 폴더 생성
    # -----------------------------
    timestamp = datetime.now().strftime("%y%m%d-%H%M")
    root_dir = Path(env_path).parent.parent / "results" / f"re-{timestamp}"
    root_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------
    # 2) 파일명 결정
    # -----------------------------
    if stage == 1:
        base = root_dir / "map"
    elif stage == 2:
        base = root_dir / f"map_{roadmap}"
    elif stage == 3:
        base = root_dir / f"map_{roadmap}_{search}"
    else:
        base = root_dir / f"stage{stage}"

    png_path = base.with_suffix(".png")
    gif_path = base.with_suffix(".gif")

    # -------------------------------------
    # 기존 env/graph/path 로드 및 그림 생성은 유지
    # -------------------------------------
    obstacles, start, goal = load_env(env_path)
    nodes_all, edges_all = ({}, [])
    path = []

    if stage >= 2 and graph_path is not None:
        nodes_all, edges_all = load_graph(graph_path)

    if stage >= 3 and path_path is not None:
        path = load_path(path_path)

    # ---------- 최종 PNG ----------
    fig, ax = plt.subplots()
    _draw_scene(
        ax,
        obstacles,
        start,
        goal,
        nodes=nodes_all if stage >= 2 else None,
        edges=edges_all if stage >= 2 else None,
        path=path if stage >= 3 else None,
    )
    ax.set_title(f"Stage {stage}")
    ax.legend()

    if save:
        fig.savefig(png_path, dpi=150)
        print(f"[visualize] Saved PNG: {png_path}")

    # -----------------------------
    # 3) 타임랩스 GIF (steps 파일 읽기)
    # -----------------------------
    steps_path = None
    if graph_path is not None:
        steps_path = Path(str(graph_path) + ".steps")

    if save and stage >= 2 and steps_path is not None and steps_path.exists() and Image is not None:
        frames = []
        nodes_present = set()
        edges_present = []

        # start/goal 노드는 처음부터 존재
        if 0 in nodes_all: nodes_present.add(0)
        if 1 in nodes_all: nodes_present.add(1)

        with steps_path.open() as f:
            in_step = False
            for line in f:
                line = line.strip()
                if not line:
                    continue

                if line == "STEP":
                    in_step = True
                    continue

                if line == "END":
                    if in_step:
                        fig_s, ax_s = plt.subplots()
                        nodes_sub = {nid: nodes_all[nid] for nid in nodes_present if nid in nodes_all}

                        _draw_scene(
                            ax_s,
                            obstacles,
                            start,
                            goal,
                            nodes=nodes_sub,
                            edges=edges_present,
                            path=None,
                        )

                        fig_s.canvas.draw()
                        w, h = fig_s.canvas.get_width_height()
                        buf = fig_s.canvas.buffer_rgba()
                        img = Image.frombuffer("RGBA", (w, h), buf, "raw", "RGBA", 0, 1)

                        frames.append(img)
                        plt.close(fig_s)

                    in_step = False
                    continue

                if line.startswith("NODE"):
                    parts = line.split()
                    nid = int(parts[1])
                    nodes_present.add(nid)

                elif line.startswith("EDGE"):
                    parts = line.split()
                    u = int(parts[1]); v = int(parts[2])
                    edges_present.append((u, v))

        if frames:
            frames[0].save(
                gif_path,
                save_all=True,
                append_images=frames[1:],
                duration=100,
                loop=0,
            )
            print(f"[visualize] Saved GIF: {gif_path}")

    plt.show()