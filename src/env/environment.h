// /home/changmin/PyProject/planning-example/src/env/environment.h
#pragma once
#include <vector>
#include "obstacle.h"

class Environment {
public:
    double world_min = 0.0;
    double world_max = 20.0;

    Vec2 start = Vec2(0.5, 0.5);
    Vec2 goal  = Vec2(19.5, 19.5);

    std::vector<Obstacle> obstacles;

    Environment() = default;

    // M개의 obstacle을 생성 (M은 1~20으로 clamp)
    void generateRandom(int M);

    bool isFree(const Vec2& p) const;
    bool segmentFree(const Vec2& a, const Vec2& b) const;

private:
    Obstacle createRandomPolygon(int vertex_count, double cx, double cy);
};
