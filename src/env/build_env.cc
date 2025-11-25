#include "environment.h"
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <seed> <num_obstacles> <out_env_file>\n";
        return 1;
    }
    unsigned int seed = std::stoul(argv[1]);
    int num_obs = std::stoi(argv[2]);
    std::string out_file = argv[3];

    Environment env(seed);
    env.generateRandom(num_obs);

    std::ofstream ofs(out_file);
    ofs << "# seed " << seed << "\n";
    ofs << env.obstacles.size() << "\n";
    for (auto &obs : env.obstacles) {
        ofs << obs.pts.size() << "\n";
        for (auto &p : obs.pts)
            ofs << p.x << " " << p.y << "\n";
    }
    ofs << env.start.x << " " << env.start.y << "\n";
    ofs << env.goal.x  << " " << env.goal.y  << "\n";
    return 0;
}
