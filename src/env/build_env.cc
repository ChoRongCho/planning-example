// /src/env/build_env.cc
#include "environment.h"
#include "rng.h"
#include <fstream>
#include <iostream>
#include <random>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <num_obstacles> [seed]\n";
        return 1;
    }

    int M = std::stoi(argv[1]);
    if (M < 0) M = 0;

    unsigned int seed;

    if (argc >= 3) {
        // Python이 seed를 줬을 때
        seed = static_cast<unsigned int>(std::stoul(argv[2]));
        std::cout << "[build_env] Using provided seed = " << seed << "\n";
    } else {
        // seed를 직접 생성
        std::random_device rd;
        seed = rd();
        std::cout << "[build_env] Generated random seed = " << seed << "\n";
    }

    RNG::seed(seed);

    Environment env;
    env.generateRandom(M);

    std::ofstream ofs("env.txt");
    ofs << "# seed " << seed << "\n";
    ofs << env.obstacles.size() << "\n";

    for (const auto& obs : env.obstacles) {
        ofs << obs.pts.size() << "\n";
        for (const auto& p : obs.pts)
            ofs << p.x << " " << p.y << "\n";
    }

    ofs << env.start.x << " " << env.start.y << "\n";
    ofs << env.goal.x  << " " << env.goal.y << "\n";

    return 0;
}
