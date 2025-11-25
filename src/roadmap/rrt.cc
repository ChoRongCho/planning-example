// /src/roadmap/rrt.cc
#include "roadmap/rrt.h"
#include "env/rng.h"
#include "roadmap/sampling.h"
#include <cmath>
#include <vector>
#include <iostream>

struct RRTNode {
    Vec2 p;
    int parent; // index in tree
};


Graph buildRRTGraph(const Environment &env,
                    int max_iter,
                    double step_size,
                    double goal_threshold,
                    double goal_bias,
                    std::ostream *log) {
    std::vector<RRTNode> tree;
    tree.push_back({env.start, -1});

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

    int goal_index = -1;

    for (int iter = 0; iter < max_iter; ++iter) {
        Vec2 sample;
        if (RNG::uniform(0.0, 1.0) < goal_bias)
            sample = env.goal;
        else
            sample = sampleUniformFree(env);

        // nearest
        int nearest = 0;
        double best = 1e9;
        for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
            double dx = tree[i].p.x - sample.x;
            double dy = tree[i].p.y - sample.y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < best) {
                best = d;
                nearest = i;
            }
        }

        Vec2 q_near = tree[nearest].p;
        double dx = sample.x - q_near.x;
        double dy = sample.y - q_near.y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-9)
            continue;

        Vec2 q_new(q_near.x + step_size * dx / len,
                   q_near.y + step_size * dy / len);
        if (!env.isFree(q_new))
            continue;
        if (!env.segmentFree(q_near, q_new))
            continue;

        tree.push_back({q_new, nearest});
        int new_idx = static_cast<int>(tree.size()) - 1;

        begin_step();
        // new_idx는 나중에 Graph에서 id가 new_idx가 된다
        log_node(new_idx, q_new);
        log_edge(nearest, new_idx);

        double d_goal = std::sqrt((q_new.x - env.goal.x) * (q_new.x - env.goal.x) +
                                  (q_new.y - env.goal.y) * (q_new.y - env.goal.y));
        if (d_goal < goal_threshold && env.segmentFree(q_new, env.goal)) {
            tree.push_back({env.goal, new_idx});
            int goal_idx = static_cast<int>(tree.size()) - 1;
            log_node(goal_idx, env.goal);
            log_edge(new_idx, goal_idx);
            goal_index = goal_idx;
            end_step();
            break;
        }

        end_step();
    }

    // Graph 변환은 기존 코드 그대로...
    Graph g;
    int N = static_cast<int>(tree.size());
    g.nodes.reserve(N);
    for (int i = 0; i < N; ++i) {
        g.nodes.push_back({i, tree[i].p});
    }
    for (int i = 0; i < N; ++i) {
        int parent = tree[i].parent;
        if (parent >= 0) {
            const Vec2 &a = tree[i].p;
            const Vec2 &b = tree[parent].p;
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            g.edges.push_back({parent, i, dist});
            g.edges.push_back({i, parent, dist});
        }
    }
    return g;
}
