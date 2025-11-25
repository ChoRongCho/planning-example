#pragma once
#include <vector>

struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

class Obstacle {
public:
    std::vector<Vec2> pts;   // Simple polygon vertices (closed implicitly)

    Obstacle() {}
    explicit Obstacle(const std::vector<Vec2>& vertices) : pts(vertices) {}

    bool contains(const Vec2& p) const;
    bool intersectsSegment(const Vec2& a, const Vec2& b) const;

private:
    bool onSegment(const Vec2& a, const Vec2& b, const Vec2& p) const;
    bool segmentsIntersect(const Vec2& p1, const Vec2& p2,
                           const Vec2& q1, const Vec2& q2) const;
};
