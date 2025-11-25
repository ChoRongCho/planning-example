// /home/changmin/PyProject/planning-example/src/env/environment.cc
#include "environment.h"
#include "rng.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
    constexpr double PI = 3.14159265358979323846;

    double polygonArea(const Obstacle& obs) {
        const auto& v = obs.pts;
        int n = static_cast<int>(v.size());
        double area = 0.0;
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            area += v[i].x * v[j].y - v[j].x * v[i].y;
        }
        return 0.5 * std::fabs(area);
    }
}

Obstacle Environment::createRandomPolygon(int vertex_count, double cx, double cy) {
    std::vector<double> angles(vertex_count);
    for (int i = 0; i < vertex_count; ++i)
        angles[i] = RNG::uniform(0.0, 2.0 * PI);
    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> pts;
    pts.reserve(vertex_count);

    // radius 범위를 적당히 잡음 (area 제약은 나중에 필터링)
    double r_min = 0.7;
    double r_max = 2.5;

    for (int i = 0; i < vertex_count; ++i) {
        double r = RNG::uniform(r_min, r_max);
        double x = cx + r * std::cos(angles[i]);
        double y = cy + r * std::sin(angles[i]);
        pts.emplace_back(x, y);
    }
    return Obstacle(pts);
}

void Environment::generateRandom(int M_input) {
    obstacles.clear();

    int M = std::max(0, std::min(M_input, 20));  // 최대 20개
    if (M_input > 20) {
        std::cerr << "[Environment] Requested " << M_input
                  << " obstacles, clamped to 20.\n";
    }
    if (M == 0) return;

    int global_attempts = 0;
    const int MAX_GLOBAL_ATTEMPTS = M * 200;

    while (static_cast<int>(obstacles.size()) < M &&
           global_attempts < MAX_GLOBAL_ATTEMPTS) {

        ++global_attempts;

        // 중심은 boundary에서 약간 떨어진 곳에서 뽑기
        double margin = 3.0;
        double cx = RNG::uniform(world_min + margin, world_max - margin);
        double cy = RNG::uniform(world_min + margin, world_max - margin);

        int vertex_count = RNG::uniformInt(4, 8);  // 4~8각형

        // 다각형 후보 여러 번 시도해서 area 조건 맞는 것 선택
        bool placed = false;
        for (int attempt = 0; attempt < 30 && !placed; ++attempt) {
            Obstacle poly = createRandomPolygon(vertex_count, cx, cy);

            // world 바깥으로 나간 vertex 있으면 reject
            bool out = false;
            for (auto &p : poly.pts) {
                if (p.x < world_min + 0.1 || p.x > world_max - 0.1 ||
                    p.y < world_min + 0.1 || p.y > world_max - 0.1) {
                    out = true; break;
                }
            }
            if (out) continue;

            double area = polygonArea(poly);
            if (area < 5.0 || area > 18.0)
                continue;

            // start / goal 포함 안되게
            if (poly.contains(start)) continue;
            if (poly.contains(goal))  continue;
            
            // ───────────── 새로 추가: 다른 obstacle들과 겹치지 않게 ─────────────
            bool overlap = false;
            for (const auto &existing : obstacles) {
                // 1) 후보 폴리곤의 edge가 기존 obstacle의 edge와 교차하는지
                int n = static_cast<int>(poly.pts.size());
                for (int i = 0; i < n && !overlap; ++i) {
                    const Vec2 &a = poly.pts[i];
                    const Vec2 &b = poly.pts[(i + 1) % n];
                    if (existing.intersectsSegment(a, b)) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) break;

                // 2) 후보 폴리곤의 vertex가 기존 obstacle 내부에 있는지
                for (const auto &p : poly.pts) {
                    if (existing.contains(p)) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) break;

                // 3) 기존 obstacle의 vertex가 후보 폴리곤 내부에 있는지
                for (const auto &p : existing.pts) {
                    if (poly.contains(p)) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) break;
            }
            if (overlap)
                continue;

            obstacles.push_back(poly);
            placed = true;
        }
    }

    if (static_cast<int>(obstacles.size()) < M) {
        std::cerr << "[Environment] Warning: requested " << M
                  << " obstacles, but only generated "
                  << obstacles.size() << ".\n";
    }
}

bool Environment::isFree(const Vec2& p) const {
    if (p.x < world_min || p.x > world_max ||
        p.y < world_min || p.y > world_max)
        return false;

    for (const auto& obs : obstacles) {
        if (obs.contains(p))
            return false;
    }
    return true;
}

bool Environment::segmentFree(const Vec2& a, const Vec2& b) const {
    for (const auto& obs : obstacles) {
        if (obs.intersectsSegment(a, b))
            return false;
    }
    return true;
}
