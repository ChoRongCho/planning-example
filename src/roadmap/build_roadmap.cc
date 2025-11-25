#include "environment.h"
#include "prm.h"       // 나중에 구현
#include "visibility.h"
#include "gvd.h"
#include "rrt.h"
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <env_file> <roadmap_type> <out_graph_file>\n";
        return 1;
    }
    std::string env_file   = argv[1];
    std::string roadmap_tp = argv[2];
    std::string out_file   = argv[3];

    // env 로드
    std::ifstream ifs(env_file);
    if (!ifs) { std::cerr << "Cannot open env_file\n"; return 1; }

    unsigned int dummy_seed;
    int M;
    std::string hash;
    ifs >> hash >> hash >> dummy_seed;   // "# seed xxx"
    ifs >> M;
    Environment env;
    env.obstacles.clear();
    for (int i=0; i<M; ++i) {
        int k; ifs >> k;
        std::vector<Vec2> pts(k);
        for (int j=0; j<k; ++j)
            ifs >> pts[j].x >> pts[j].y;
        env.obstacles.emplace_back(pts);
    }
    ifs >> env.start.x >> env.start.y;
    ifs >> env.goal.x  >> env.goal.y;

    // roadmap 생성
    Graph g;  // 공통 그래프 타입 만든다고 가정

    if (roadmap_tp == "prm_random") {
        g = buildPRM(env, /*n_samples=*/400, /*sampler=*/SamplerType::RANDOM);
    } else if (roadmap_tp == "visibility") {
        g = buildVisibility(env);
    } else if (roadmap_tp == "gvd") {
        g = buildGVD(env);
    } else if (roadmap_tp == "rrt") {
        g = buildRRTGraph(env);
    } else {
        std::cerr << "Unknown roadmap_type\n";
        return 1;
    }

    // graph.txt 저장
    std::ofstream ofs(out_file);
    ofs << g.nodes.size() << "\n";
    for (auto &n : g.nodes)
        ofs << n.id << " " << n.p.x << " " << n.p.y << "\n";
    ofs << g.edges.size() << "\n";
    for (auto &e : g.edges)
        ofs << e.u << " " << e.v << " " << e.w << "\n";

    return 0;
}
