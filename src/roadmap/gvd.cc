// /src/roadmap/gvd.cc
#include "roadmap/gvd.h"
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "thirdparty/jc_voronoi.h"
#include "thirdparty/jc_voronoi_clip.h"

// --- 작은 유틸들 ---
static double sqr(double x) { return x * x; }

struct PointKey {
    double x, y;
};

struct PointKeyHash {
    std::size_t operator()(const PointKey &k) const noexcept {
        // 간단한 해시 (충분히 ok)
        auto h1 = std::hash<long long>{}(static_cast<long long>(k.x * 1e6));
        auto h2 = std::hash<long long>{}(static_cast<long long>(k.y * 1e6));
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

struct PointKeyEq {
    bool operator()(const PointKey &a, const PointKey &b) const noexcept {
        const double eps = 1e-6;
        return std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps;
    }
};

static int add_node_if_new(Graph &g,
                           std::unordered_map<PointKey, int, PointKeyHash, PointKeyEq> &map,
                           const jcv_point &p,
                           std::ostream *out)
{
    PointKey key{static_cast<double>(p.x), static_cast<double>(p.y)};
    auto it = map.find(key);
    if (it != map.end())
        return it->second;

    int id = static_cast<int>(g.nodes.size());
    Vec2 v(key.x, key.y);
    g.nodes.push_back({id, v});
    map.emplace(key, id);

    if (out)
        (*out) << "NODE " << id << " " << v.x << " " << v.y << "\n";

    return id;
}

// --- GVD Voronoi 구현 ---

Graph buildGVDVoronoi(const Environment &env,
                      std::ostream *out)
{
    Graph g;
    g.nodes.clear();
    g.edges.clear();

    // 0,1번은 항상 start / goal
    g.nodes.push_back({0, env.start});
    g.nodes.push_back({1, env.goal});

    if (out) {
        (*out) << "STEP\n";
        (*out) << "NODE 0 " << env.start.x << " " << env.start.y << "\n";
        (*out) << "NODE 1 " << env.goal.x  << " " << env.goal.y  << "\n";
        (*out) << "END\n";
    }

    // 1) site 집합 만들기: 각 장애물 버텍스를 하나의 site로 둔다
    std::vector<jcv_point> sites;
    std::vector<int>       site_obs_id;   // 이 site가 어떤 obstacle에 속하는지
    for (int oi = 0; oi < static_cast<int>(env.obstacles.size()); ++oi) {
        const Obstacle &obs = env.obstacles[oi];
        for (const Vec2 &p : obs.pts) {
            jcv_point s;
            s.x = static_cast<jcv_real>(p.x);
            s.y = static_cast<jcv_real>(p.y);
            sites.push_back(s);
            site_obs_id.push_back(oi);
        }
    }
    const int num_sites = static_cast<int>(sites.size());
    if (num_sites == 0) {
        // 장애물이 하나도 없으면 그냥 start-goal 직선만 쓰자
        double dx = env.goal.x - env.start.x;
        double dy = env.goal.y - env.start.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        g.edges.push_back({0, 1, dist});
        g.edges.push_back({1, 0, dist});
        return g;
    }

    // 2) 다이어그램 생성 (월드 박스로 클리핑)
    jcv_rect rect;
    rect.min.x = env.world_min;
    rect.min.y = env.world_min;
    rect.max.x = env.world_max;
    rect.max.y = env.world_max;

    jcv_diagram diagram;
    std::memset(&diagram, 0, sizeof(diagram));
    jcv_diagram_generate(num_sites,
                         sites.data(),
                         &rect,
                         nullptr,   // clipping 없음
                         &diagram);

    // 3) Voronoi edge들을 순회하면서,
    //    서로 다른 obstacle에 속한 site 쌍이 만드는 edge만 skeleton으로 쓴다
    std::unordered_map<PointKey, int, PointKeyHash, PointKeyEq> point_to_id;

    if (out) (*out) << "STEP\n";

    const jcv_edge *edge = jcv_diagram_get_edges(&diagram);
    while (edge) {

        // (1) site 포인터 NULL 체크 필수
        if (!edge->sites[0] || !edge->sites[1]) {
            edge = jcv_diagram_get_next_edge(edge);
            continue;
        }

        int si0 = edge->sites[0]->index;
        int si1 = edge->sites[1]->index;

        if (si0 < 0 || si0 >= num_sites ||
            si1 < 0 || si1 >= num_sites) {
            edge = jcv_diagram_get_next_edge(edge);
            continue;
        }

        int obs0 = site_obs_id[si0];
        int obs1 = site_obs_id[si1];

        // 같은 obstacle 제외
        if (obs0 == obs1) {
            edge = jcv_diagram_get_next_edge(edge);
            continue;
        }

        // (2) infinite edge check
        const jcv_point &p0 = edge->pos[0];
        const jcv_point &p1 = edge->pos[1];

        if (sqr(p0.x - p1.x) + sqr(p0.y - p1.y) < 1e-12) {
            edge = jcv_diagram_get_next_edge(edge);
            continue;
        }

        // (3) free space check
        Vec2 v0(p0.x, p0.y);
        Vec2 v1(p1.x, p1.y);

        if (!env.isFree(v0) || !env.isFree(v1) ||
            !env.segmentFree(v0, v1)) {
            edge = jcv_diagram_get_next_edge(edge);
            continue;
        }

        // (4) 노드 생성
        int id0 = add_node_if_new(g, point_to_id, p0, out);
        int id1 = add_node_if_new(g, point_to_id, p1, out);

        double dx = v0.x - v1.x;
        double dy = v0.y - v1.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        g.edges.push_back({id0, id1, dist});
        g.edges.push_back({id1, id0, dist});

        if (out)
            (*out) << "EDGE " << id0 << " " << id1 << "\n";

        edge = jcv_diagram_get_next_edge(edge);
    }

    if (out) (*out) << "END\n";

    // 4) start / goal 을 skeleton에 붙인다
    auto connect_anchor = [&](int anchor_id, const Vec2 &p) {
        const int N = static_cast<int>(g.nodes.size());
        double best_dist = 1e9;
        int best_idx = -1;
        for (int i = 2; i < N; ++i) {  // 0,1은 start/goal 자신
            const Vec2 &q = g.nodes[i].p;
            double dx = q.x - p.x;
            double dy = q.y - p.y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < best_dist && env.segmentFree(p, q)) {
                best_dist = d;
                best_idx = i;
            }
        }
        if (best_idx >= 0) {
            g.edges.push_back({anchor_id, best_idx, best_dist});
            g.edges.push_back({best_idx, anchor_id, best_dist});
            if (out) {
                (*out) << "STEP\n";
                (*out) << "EDGE " << anchor_id << " " << best_idx << "\n";
                (*out) << "END\n";
            }
        }
    };

    connect_anchor(0, env.start);
    connect_anchor(1, env.goal);

    jcv_diagram_free(&diagram);
    return g;
}
