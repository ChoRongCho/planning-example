// /src/roadmap/graph.h
#pragma once
#include <vector>
#include "env/obstacle.h"  // for Vec2

struct RoadmapNode {
    int id;
    Vec2 p;
};

struct RoadmapEdge {
    int u;
    int v;
    double w;
};

struct Graph {
    std::vector<RoadmapNode> nodes;
    std::vector<RoadmapEdge> edges;
};
