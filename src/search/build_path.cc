#include "environment.h"
#include "search.h"   // A*, BFS, GBFS 등 공통 인터페이스
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <env_file> <graph_file> <search_type> <out_path_file>\n";
        return 1;
    }
    std::string env_file   = argv[1];
    std::string graph_file = argv[2];
    std::string search_tp  = argv[3];
    std::string out_file   = argv[4];

    // start/goal만 env에서 읽기
    std::ifstream ifs(env_file);
    if (!ifs) { std::cerr << "Cannot open env_file\n"; return 1; }

    unsigned int dummy_seed;
    int M;
    std::string hash;
    ifs >> hash >> hash >> dummy_seed;
    ifs >> M;
    for (int i=0; i<M; ++i) {
        int k; ifs >> k;
        double x,y;
        for (int j=0; j<k; ++j) ifs >> x >> y;
    }
    Environment env;
    ifs >> env.start.x >> env.start.y;
    ifs >> env.goal.x  >> env.goal.y;

    // graph 로드
    Graph g;
    std::ifstream gfs(graph_file);
    if (!gfs) { std::cerr << "Cannot open graph_file\n"; return 1; }

    int N; gfs >> N;
    g.nodes.resize(N);
    for (int i=0; i<N; ++i) {
        int id; double x,y;
        gfs >> id >> x >> y;
        g.nodes[id].id = id;
        g.nodes[id].p  = Vec2(x,y);
    }
    int E; gfs >> E;
    g.edges.resize(E);
    for (int i=0; i<E; ++i)
        gfs >> g.edges[i].u >> g.edges[i].v >> g.edges[i].w;

    // start/goal node 찾기 (0/1로 고정해도 되고, 좌표 match해도 됨)
    int start_id = 0;
    int goal_id  = 1;

    auto algo = parseSearchType(search_tp);  // enum 리턴한다고 가정
    auto path = runSearch(g, start_id, goal_id, algo);

    std::ofstream ofs(out_file);
    ofs << path.size() << "\n";
    for (int idx : path)
        ofs << g.nodes[idx].p.x << " " << g.nodes[idx].p.y << "\n";

    return 0;
}
