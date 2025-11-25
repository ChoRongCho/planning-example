import argparse


def parse_args():
    parser = argparse.ArgumentParser()

    # --- 어떤 단계까지 실행할지: env / map / search ---
    stage_group = parser.add_mutually_exclusive_group()
    stage_group.add_argument("--env",action="store_true",help="Environment only (env.txt)")
    stage_group.add_argument("--map", action="store_true", help="Environment + Roadmap (env.txt + graph.txt)")
    stage_group.add_argument("--search", action="store_true", help="Environment + Roadmap + Path (env.txt + graph.txt + path.txt)")
    
    parser.add_argument( "--num_obstacles", type=int, default=10, help="The number of obstacle for environment (default: 10, max 20)")
    # seed는 나중에 쓸 수 있게만 남겨두고, 현재 build_env는 내부에서 random_device 사용
    parser.add_argument( "--seed", type=int, default=None, help="(Optional) Random seed")
    parser.add_argument( "--roadmap", type=str, default="prm_random", 
                        choices=["prm_random", "prm_halton", "prm_sobol", "visibility", "gvd", "rrt"], help="Roadmap algorithm")
    parser.add_argument( "--search_method", type=str, default="astar", 
                        choices=["bfs", "dfs", "gbfs", "astar", "wastar"], help="Path finding algorithm")
    
    parser.add_argument( "--no_vis", action="store_true", help="No visualize")
    parser.add_argument("--save", action="store_false", default=True, help="Save PNG and GIF")

    return parser.parse_args()


def get_stage(args) -> int:
    if args.env: return 1
    if args.map: return 2
    if args.search: return 3
    return 3
