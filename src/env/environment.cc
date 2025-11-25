#include "environment.h"
#include "rng.h"
#include <cmath>

Environment::Environment(unsigned int seed) {
    RNG::seed(seed);
}

Obstacle Environment::createRandomPolygon(int vertex_count, double cx, double cy) {
    std::vector<Vec2> pts;
    std::vector<double> angles(vertex_count);

    for (int i=0; i<vertex_count; i++)
        angles[i] = RNG::uniform(0, 2*M_PI);

    std::sort(angles.begin(), angles.end());

    for (int i=0; i<vertex_count; i++) {
        double r = RNG::uniform(0.5, 2.5);
        pts.emplace_back(
            cx + r * std::cos(angles[i]),
            cy + r * std::sin(angles[i])
        );
    }
    return Obstacle(pts);
}

void Environment::generateRandom(int M) {
    obstacles.clear();

    int tries = 0;
    while ((int)obstacles.size() < M && tries < M*20) {
        tries++;

        double cx = RNG::uniform(world_min+2.0, world_max-2.0);
        double cy = RNG::uniform(world_min+2.0, world_max-2.0);

        int verts = 4 + (int)(RNG::uniform(0,1) * 5);
        Obstacle poly = createRandomPolygon(verts, cx, cy);

        if (poly.contains(start)) continue;
        if (poly.contains(goal)) continue;

        bool out = false;
        for (auto &p : poly.pts) {
            if (p.x < world_min+0.1 || p.x > world_max-0.1 ||
                p.y < world_min+0.1 || p.y > world_max-0.1) {
                out = true; break;
            }
        }
        if (out) continue;

        obstacles.push_back(poly);
    }
}

bool Environment::isFree(const Vec2& p) const {
    if (p.x < world_min || p.x > world_max ||
        p.y < world_min || p.y > world_max) return false;

    for (auto &obs : obstacles)
        if (obs.contains(p)) return false;

    return true;
}

bool Environment::segmentFree(const Vec2& a, const Vec2& b) const {
    for (auto &obs : obstacles)
        if (obs.intersectsSegment(a,b)) return false;

    return true;
}
