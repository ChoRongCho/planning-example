// /src/search/build_path.cc
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "env/environment.h"   // start/goal 좌표 정의용 (0,1번 노드지만 타입 맞추려고)
#include "roadmap/graph.h"
#include "search/search.h"

using namespace std;

// graph.txt 로부터 Graph 읽기
static void loadGraphFromFile(const std::string &filename, Graph &g) {
    ifstream ifs(filename);
    if (!ifs) {
        throw runtime_error("Cannot open graph file: " + filename);
    }

    int N;
    ifs >> N;
    g.nodes.clear();
    g.edges.clear();
    g.nodes.reserve(N);

    for (int i = 0; i < N; ++i) {
        int id;
        double x, y;
        ifs >> id >> x >> y;
        Vec2 p(x, y);
        g.nodes.push_back({id, p});
    }

    int M;
    ifs >> M;
    g.edges.reserve(M);
    for (int i = 0; i < M; ++i) {
        int u, v;
        double w;
        ifs >> u >> v >> w;
        g.edges.push_back({u, v, w});
    }
}

// path.txt 로 저장
static void savePathToFile(const std::string &filename,
                           const Graph &g,
                           const std::vector<int> &path_ids)
{
    ofstream ofs(filename);
    if (!ofs) {
        throw runtime_error("Cannot open path file for writing: " + filename);
    }
    ofs << path_ids.size() << "\n";
    for (int id : path_ids) {
        const Vec2 &p = g.nodes[id].p;
        ofs << p.x << " " << p.y << "\n";
    }
}

int main(int argc, char **argv) {
    if (argc < 5) {
        cerr << "Usage: " << argv[0]
             << " <env_file> <graph_file> <search_type> <out_path_file>\n";
        cerr << " search_type: bfs | dfs | gbfs | astar | wastar\n";
        return 1;
    }

    std::string env_file   = argv[1]; // 현재는 직접 쓰지는 않지만, 인터페이스 용으로 유지
    std::string graph_file = argv[2];
    std::string search_str = argv[3];
    std::string out_file   = argv[4];

    Graph g;
    try {
        loadGraphFromFile(graph_file, g);
    } catch (const std::exception &e) {
        cerr << "[build_path] Error loading graph: " << e.what() << "\n";
        return 1;
    }

    if (g.nodes.size() < 2) {
        cerr << "[build_path] Graph has fewer than 2 nodes\n";
        return 1;
    }

    int start_id = 0; // build_roadmap에서 항상 start=0, goal=1 로 생성했다고 가정
    int goal_id  = 1;

    SearchType type;
    try {
        type = parse_search_type(search_str);
    } catch (const std::exception &e) {
        cerr << "[build_path] " << e.what() << "\n";
        return 1;
    }

    std::vector<int> path_ids;
    auto t0 = std::chrono::steady_clock::now();
    bool ok = run_search(g, start_id, goal_id, type, path_ids);
    auto t1 = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
            .count();

    if (!ok) {
        cerr << "[build_path] No path found by " << search_str << "\n";
        return 1;
    }

    cout << "[build_path] search=" << search_str
         << " | path_len=" << path_ids.size()
         << " | time=" << elapsed << " s\n";

    try {
        savePathToFile(out_file, g, path_ids);
    } catch (const std::exception &e) {
        cerr << "[build_path] Error saving path: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
