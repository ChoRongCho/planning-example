// /src/roadmap/build_roadmap.cc
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>                 // for timing
#include "env/environment.h"
#include "env/rng.h"
#include "roadmap/graph.h"
#include "roadmap/prm.h"
#include "roadmap/visibility.h"
#include "roadmap/gvd.h"
#include "roadmap/rrt.h"

static void loadEnvironmentFromFile(const std::string &filename, Environment &env, unsigned int &seed_out) {
    std::ifstream ifs(filename);
    if (!ifs) {
        throw std::runtime_error("Cannot open env file: " + filename);
    }

    // First line: "# seed <value>"
    std::string hash, word;
    ifs >> hash >> word >> seed_out; // "# seed 12345"

    int M;
    ifs >> M;
    env.obstacles.clear();
    for (int i = 0; i < M; ++i) {
        int k;
        ifs >> k;
        std::vector<Vec2> pts(k);
        for (int j = 0; j < k; ++j) {
            ifs >> pts[j].x >> pts[j].y;
        }
        env.obstacles.emplace_back(pts);
    }

    ifs >> env.start.x >> env.start.y;
    ifs >> env.goal.x  >> env.goal.y;
}

static void saveGraphToFile(const std::string &filename, const Graph &g) {
    std::ofstream ofs(filename);
    if (!ofs) {
        throw std::runtime_error("Cannot open graph file for writing: " + filename);
    }

    ofs << g.nodes.size() << "\n";
    for (const auto &n : g.nodes) {
        ofs << n.id << " " << n.p.x << " " << n.p.y << "\n";
    }

    ofs << g.edges.size() << "\n";
    for (const auto &e : g.edges) {
        ofs << e.u << " " << e.v << " " << e.w << "\n";
    }
}



int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <env_file> <roadmap_type> <out_graph_file>\n";
        std::cerr << "roadmap_type: prm_random | prm_halton | prm_sobol | visibility | gvd | rrt\n";
        return 1;
    }

    std::string env_file   = argv[1];
    std::string roadmap_tp = argv[2];
    std::string out_file   = argv[3];

    Environment env;
    unsigned int env_seed = 0;
    try {
        loadEnvironmentFromFile(env_file, env, env_seed);
    } catch (const std::exception &e) {
        std::cerr << "[build_roadmap] Error: " << e.what() << "\n";
        return 1;
    }

    // Use environment seed + roadmap type hash to seed RNG for reproducible PRM/RRT
    unsigned int seed = env_seed;
    for (char c : roadmap_tp) {
        seed = seed * 131u + static_cast<unsigned char>(c);
    }
    RNG::seed(seed);

    // step 로그 파일: graph.txt.steps 같이 옆에 만든다
    std::string steps_file = out_file + ".steps";
    std::ofstream steps_ofs(steps_file);
    if (!steps_ofs) {
        std::cerr << "[build_roadmap] Warning: cannot open steps log: " << steps_file << "\n";
    }

    Graph g;

    try {
        auto t_start = std::chrono::high_resolution_clock::now();

        std::ostream *log = steps_ofs ? &steps_ofs : nullptr;

        if (roadmap_tp == "prm_random" ||
            roadmap_tp == "prm_halton" ||
            roadmap_tp == "prm_sobol") {

            SamplerType sampler = samplerFromString(roadmap_tp);
            g = buildPRM(env, /*n_samples=*/400, sampler, /*radius=*/2.0, log);

        } else if (roadmap_tp == "visibility") {
            g = buildVisibilityGraph(env, log);

        } else if (roadmap_tp == "gvd") {
            g = buildGVDLike(env, /*grid_res=*/40, /*band_min=*/0.3,
                             /*band_max=*/1.5, /*connect_radius=*/2.0, log);

        } else if (roadmap_tp == "rrt") {
            g = buildRRTGraph(env, /*max_iter=*/3000, /*step_size=*/0.5,
                              /*goal_threshold=*/0.5, /*goal_bias=*/0.05, log);

        } else {
            std::cerr << "[build_roadmap] Unknown roadmap_type: " << roadmap_tp << "\n";
            return 1;
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_s =
            std::chrono::duration<double>(t_end - t_start).count();

        std::cout << "[build_roadmap] type=" << roadmap_tp
                  << " | nodes=" << g.nodes.size()
                  << " | edges=" << g.edges.size()
                  << " | build_time=" << elapsed_s << " s"
                  << std::endl;

        saveGraphToFile(out_file, g);
    } catch (const std::exception &e) {
        std::cerr << "[build_roadmap] Error while building roadmap: " << e.what() << "\n";
        return 1;
    }

    return 0;
}