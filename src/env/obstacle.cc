// /home/changmin/PyProject/planning-example/src/env/obstacle.cc
#include "obstacle.h"
#include <cmath>
#include <algorithm>

namespace {
    double cross(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
        // (b - a) x (c - a)
        return (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
    }
}

bool Obstacle::onSegment(const Vec2& a, const Vec2& b, const Vec2& p) const {
    if (std::fabs(cross(a,b,p)) > 1e-9) return false;
    return (p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
            p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y));
}

bool Obstacle::segmentsIntersect(const Vec2& p1, const Vec2& p2,
                                 const Vec2& q1, const Vec2& q2) const {
    auto ccw = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
        double cr = cross(a,b,c);
        if (cr > 1e-9) return 1;
        if (cr < -1e-9) return -1;
        return 0;
    };

    int d1 = ccw(p1,p2,q1);
    int d2 = ccw(p1,p2,q2);
    int d3 = ccw(q1,q2,p1);
    int d4 = ccw(q1,q2,p2);

    if (d1*d2 < 0 && d3*d4 < 0) return true;
    if (d1 == 0 && onSegment(p1,p2,q1)) return true;
    if (d2 == 0 && onSegment(p1,p2,q2)) return true;
    if (d3 == 0 && onSegment(q1,q2,p1)) return true;
    if (d4 == 0 && onSegment(q1,q2,p2)) return true;

    return false;
}

bool Obstacle::contains(const Vec2& p) const {
    // ray casting
    bool inside = false;
    int n = static_cast<int>(pts.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 &a = pts[j];
        const Vec2 &b = pts[i];
        bool intersect = ((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-12) + a.x);
        if (intersect) inside = !inside;
    }
    return inside;
}

bool Obstacle::intersectsSegment(const Vec2& a, const Vec2& b) const {
    int n = static_cast<int>(pts.size());
    for (int i = 0; i < n; ++i) {
        const Vec2& c = pts[i];
        const Vec2& d = pts[(i + 1) % n];
        if (segmentsIntersect(a,b,c,d))
            return true;
    }
    return false;
}
