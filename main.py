import argparse
import subprocess
from pathlib import Path
from scripts.visualize import visualize

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"

def run_cmd(cmd):
    print("[RUN]", " ".join(map(str, cmd)))
    subprocess.run(cmd, check=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", type=int, default=3,
                        choices=[1,2,3],
                        help="1: env only, 2: env+roadmap, 3: env+roadmap+path")
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--num_obstacles", type=int, default=10)

    parser.add_argument("--roadmap", type=str, default="prm_random",
                        choices=["prm_random", "prm_halton", "prm_sobol",
                                 "visibility", "gvd", "rrt"])
    parser.add_argument("--search", type=str, default="astar",
                        choices=["bfs", "dfs", "gbfs", "astar", "wastar"])

    parser.add_argument("--no_vis", action="store_true",
                        help="Do not visualize, only generate data.")
    args = parser.parse_args()

    RESULTS.mkdir(exist_ok=True)

    env_file   = RESULTS / "env.txt"
    graph_file = RESULTS / "graph.txt"
    path_file  = RESULTS / "path.txt"

    # 1단계: env 생성
    if args.stage >= 1:
        run_cmd([
            str(ROOT / "build_env"),
            str(args.seed),
            str(args.num_obstacles),
            str(env_file)
        ])

    # 2단계: roadmap 생성
    if args.stage >= 2:
        run_cmd([
            str(ROOT / "build_roadmap"),
            str(env_file),
            args.roadmap,
            str(graph_file)
        ])

    # 3단계: path 생성
    if args.stage >= 3:
        run_cmd([
            str(ROOT / "build_path"),
            str(env_file),
            str(graph_file),
            args.search,
            str(path_file)
        ])

    if not args.no_vis:
        visualize(stage=args.stage,
                  env_path=env_file,
                  graph_path=graph_file,
                  path_path=path_file)

if __name__ == "__main__":
    main()
