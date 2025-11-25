import subprocess
import os
from pathlib import Path
from scripts.visualize import visualize
from scripts.utils import parse_args, get_stage

ROOT = Path(__file__).resolve().parent
BIN = os.path.join(ROOT, "build", "bin")
RESULTS = ROOT / "results"


def run_cmd(cmd, cwd=None):
    print("[RUN]", " ".join(map(str, cmd)), f"(cwd={cwd})" if cwd else "")
    subprocess.run(cmd, check=True, cwd=cwd)


def main():
    args = parse_args()
    stage = get_stage(args)

    RESULTS.mkdir(exist_ok=True)

    env_file = RESULTS / "env.txt"
    graph_file = RESULTS / "graph.txt"
    path_file = RESULTS / "path.txt"

    # Stage 1: Environment generation
    if stage >= 1:
        cmd = [os.path.join(BIN, "build_env"), str(args.num_obstacles)]
        if args.seed is not None:
            cmd.append(str(args.seed))       
            
        run_cmd(cmd, cwd=RESULTS)

    # Stage 2: Roadmap generation
    if stage >= 2:
        run_cmd(
            [
                os.path.join(BIN, "build_roadmap"),
                str(env_file),
                args.roadmap,
                str(graph_file),
            ]
        )

    # Stage 3: Path search
    if stage >= 3:
        run_cmd(
            [
                os.path.join(BIN, "build_path"),
                str(env_file),
                str(graph_file),
                args.search_method,
                str(path_file),
            ]
        )

    if not args.no_vis:
        visualize(
            stage=stage,
            env_path=env_file,
            graph_path=graph_file,
            path_path=path_file,
            save=args.save, 
        )


if __name__ == "__main__":
    main()
