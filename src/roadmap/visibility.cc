// /src/roadmap/visibility.cc
#include "roadmap/visibility.h"
#include <cmath>

Graph buildVisibilityGraph(const Environment &env,
                           std::ostream *out) {
    Graph g;
    g.nodes.clear();
    g.edges.clear();

    // Start & goal
    g.nodes.push_back({0, env.start});
    g.nodes.push_back({1, env.goal});

    auto begin_step = [&]() {
        if (out) (*out) << "STEP\n";
    };
    auto end_step = [&]() {
        if (out) (*out) << "END\n";
        if (out) out->flush();
    };
    auto log_node = [&](int id, const Vec2 &p) {
        if (out) (*out) << "NODE " << id << " " << p.x << " " << p.y << "\n";
    };
    auto log_edge = [&](int u, int v) {
        if (out) (*out) << "EDGE " << u << " " << v << "\n";
    };

    // 장애물 vertex를 하나씩 추가하면서, 기존 노드와 visibility edge 연결
    for (const auto &obs : env.obstacles) {
        for (const auto &p : obs.pts) {
            // ⚠️ 장애물 꼭짓점은 경계에 있기 때문에 isFree(p)로 걸러버리면 안 됨
            // if (!env.isFree(p))
            //     continue;

            int id = static_cast<int>(g.nodes.size());
            g.nodes.push_back({id, p});

            begin_step();
            log_node(id, p);

            // 이전에 존재하던 모든 노드와 visibility 체크
            for (int j = 0; j < id; ++j) {
                const Vec2 &a = g.nodes[j].p;
                // 선분이 어떤 장애물도 가로지르지 않을 때만 연결
                if (!env.segmentFree(a, p))
                    continue;

                double dx = a.x - p.x;
                double dy = a.y - p.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                g.edges.push_back({j, id, dist});
                g.edges.push_back({id, j, dist});
                log_edge(j, id);
            }

            end_step();
        }
    }

    return g;
}
