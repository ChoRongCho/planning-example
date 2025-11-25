// /home/changmin/PyProject/planning-example/src/env/obstacle.h
#pragma once
#include <vector>

struct Vec2 {
    double x, y;
    Vec2() : x(0.0), y(0.0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

class Obstacle {
public:
    std::vector<Vec2> pts;   // simple polygon (Jordan curve)

    Obstacle() = default;
    explicit Obstacle(const std::vector<Vec2>& vertices) : pts(vertices) {}

    // 점이 다각형 내부에 있는지 (edge 위 포함해도 됨)
    bool contains(const Vec2& p) const;

    // 선분 (a,b)가 이 다각형의 어떤 edge와도 교차하는지
    bool intersectsSegment(const Vec2& a, const Vec2& b) const;

private:
    bool onSegment(const Vec2& a, const Vec2& b, const Vec2& p) const;
    bool segmentsIntersect(const Vec2& p1, const Vec2& p2,
                           const Vec2& q1, const Vec2& q2) const;
};
