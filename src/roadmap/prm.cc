// /src/roadmap/prm.cc
#include "roadmap/prm.h"
#include "roadmap/sampling.h"
#include <cmath>
#include <stdexcept>
#include <ostream>
#include <iostream>


SamplerType samplerFromString(const std::string &name) {
    if (name == "prm_random")
        return SamplerType::RANDOM;
    if (name == "prm_halton")
        return SamplerType::HALTON;
    if (name == "prm_sobol")
        return SamplerType::SOBOL;
    throw std::runtime_error("Unknown PRM sampler: " + name);
}

Graph buildPRM(const Environment &env, int n_samples,
               SamplerType sampler, double radius,
               std::ostream *log) {
    Graph g;
    g.nodes.clear();
    g.edges.clear();

    // start, goal 먼저 추가
    g.nodes.push_back({0, env.start});
    g.nodes.push_back({1, env.goal});

    int step = 0;
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

    // 샘플링 루프
    for (int i = 0; i < n_samples; ++i) {
        Vec2 p;
        if (sampler == SamplerType::RANDOM)
            p = sampleUniformFree(env);
        else if (sampler == SamplerType::HALTON)
            p = sampleHaltonFree(env, static_cast<std::size_t>(i + 1));
        else
            p = sampleSobolFree(env, static_cast<std::size_t>(i + 1));

        // 새 노드 id
        int id = static_cast<int>(g.nodes.size());
        // 일단 free 공간이니까 바로 추가 (충돌은 segment에서만 검사)
        g.nodes.push_back({id, p});

        begin_step();
        log_node(id, p);

        // 기존 모든 노드와 연결 시도
        for (int j = 0; j < id; ++j) {
            const Vec2 &a = g.nodes[j].p;
            const Vec2 &b = p;
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > radius)
                continue;
            if (!env.segmentFree(a, b))
                continue;
            g.edges.push_back({j, id, dist});
            g.edges.push_back({id, j, dist});
            log_edge(j, id);
        }

        end_step();
        ++step;
    }

    return g;
}
