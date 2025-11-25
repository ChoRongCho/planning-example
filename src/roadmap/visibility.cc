// /src/roadmap/visibility.cc
#include "roadmap/visibility.h"
#include <cmath>
#include <ostream>

// 간단한 geometry 유틸
static bool almost_equal(const Vec2 &a, const Vec2 &b, double eps = 1e-8) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy < eps * eps;
}

static double cross(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
    // (b - a) x (c - a)
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static bool onSegment(const Vec2 &a, const Vec2 &b, const Vec2 &p) {
    // p가 [a,b] 구간 위에 있는지 (collinear 가정)
    return (std::min(a.x, b.x) - 1e-9 <= p.x && p.x <= std::max(a.x, b.x) + 1e-9 &&
            std::min(a.y, b.y) - 1e-9 <= p.y && p.y <= std::max(a.y, b.y) + 1e-9);
}

static bool segmentsIntersect(const Vec2 &p1, const Vec2 &p2,
                              const Vec2 &q1, const Vec2 &q2) {
    double c1 = cross(p1, p2, q1);
    double c2 = cross(p1, p2, q2);
    double c3 = cross(q1, q2, p1);
    double c4 = cross(q1, q2, p2);

    if ((c1 == 0 && onSegment(p1, p2, q1)) ||
        (c2 == 0 && onSegment(p1, p2, q2)) ||
        (c3 == 0 && onSegment(q1, q2, p1)) ||
        (c4 == 0 && onSegment(q1, q2, p2)))
        return true;

    return (c1 * c2 < 0 && c3 * c4 < 0);
}

// visibility 전용: p-q 선분이 장애물 내부를 가로지르는지만 체크
// - 장애물 edge와 endpoint에서 만나는 것은 허용
static bool isVisibleSegment(const Environment &env, const Vec2 &p, const Vec2 &q) {
    for (const auto &obs : env.obstacles) {
        int n = static_cast<int>(obs.pts.size());
        for (int i = 0; i < n; ++i) {
            const Vec2 &a = obs.pts[i];
            const Vec2 &b = obs.pts[(i + 1) % n];

            // p 또는 q가 이 edge의 endpoint와 같은 경우는 경계 공유이므로 무시
            if (almost_equal(p, a) || almost_equal(p, b) ||
                almost_equal(q, a) || almost_equal(q, b)) {
                continue;
            }

            if (segmentsIntersect(p, q, a, b)) {
                // 내부에서 교차하는 경우 → 가시성 없음
                return false;
            }
        }
    }
    return true;
}

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

    // 1) 장애물 vertex들을 순서대로 노드로 추가
    for (const auto &obs : env.obstacles) {
        for (const auto &p : obs.pts) {
            int id = static_cast<int>(g.nodes.size());
            g.nodes.push_back({id, p});

            begin_step();
            log_node(id, p);

            // 2) 기존 노드들과 visibility check
            for (int j = 0; j < id; ++j) {
                const Vec2 &a = g.nodes[j].p;

                // 선분 내부에서 장애물과 교차하면 제외, endpoint at boundary는 허용
                if (!isVisibleSegment(env, a, p))
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
