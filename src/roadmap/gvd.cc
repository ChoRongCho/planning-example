// /src/roadmap/gvd.cc
#include "roadmap/gvd.h"
#include <cmath>


Graph buildGVDLike(const Environment &env,
                   int grid_resolution,
                   double band_min,
                   double band_max,
                   double connect_radius,
                   std::ostream *log) {
    Graph g;
    g.nodes.clear();
    g.edges.clear();

    // Start & goal
    g.nodes.push_back({0, env.start});
    g.nodes.push_back({1, env.goal});

    auto begin_step = [&]() {
        if (log) *log << "STEP\n";
    };
    auto end_step = [&]() {
        if (log) *log << "END\n";
        if (log) log->flush();
    };
    auto log_node = [&](int id, const Vec2 &p) {
        if (log) *log << "NODE " << id << " " << p.x << " " << p.y << "\n";
    };
    auto log_edge = [&](int u, int v) {
        if (log) *log << "EDGE " << u << " " << v << "\n";
    };

    double step = (env.world_max - env.world_min) / grid_resolution;

    for (int i = 0; i <= grid_resolution; ++i) {
        for (int j = 0; j <= grid_resolution; ++j) {
            Vec2 p(env.world_min + i * step,
                   env.world_min + j * step);
            if (!env.isFree(p))
                continue;

            double d_min = 1e9;
            for (const auto &obs : env.obstacles) {
                int n = static_cast<int>(obs.pts.size());
                for (int k = 0; k < n; ++k) {
                    const Vec2 &a = obs.pts[k];
                    const Vec2 &b = obs.pts[(k + 1) % n];
                    double dx1 = p.x - a.x;
                    double dy1 = p.y - a.y;
                    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
                    double dx2 = p.x - b.x;
                    double dy2 = p.y - b.y;
                    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
                    d_min = std::min(d_min, std::min(d1, d2));
                }
            }

            if (d_min <= band_min || d_min >= band_max)
                continue;

            int id = static_cast<int>(g.nodes.size());
            g.nodes.push_back({id, p});

            begin_step();
            log_node(id, p);

            // connect to nearby nodes
            for (int j2 = 0; j2 < id; ++j2) {
                const Vec2 &q = g.nodes[j2].p;
                double dx = p.x - q.x;
                double dy = p.y - q.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist > connect_radius)
                    continue;
                if (!env.segmentFree(p, q))
                    continue;
                g.edges.push_back({j2, id, dist});
                g.edges.push_back({id, j2, dist});
                log_edge(j2, id);
            }

            end_step();
        }
    }

    return g;
}
