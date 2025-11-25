// /src/search/search.h
#pragma once

#include <string>
#include <vector>
#include "roadmap/graph.h"

// Supported search types
enum class SearchType {
    BFS,
    DFS,
    GBFS,
    ASTAR,
    WASTAR
};

// Parse string like "bfs", "dfs", "gbfs", "astar", "wastar"
SearchType parse_search_type(const std::string &name);

// Run search on a given graph from start_id to goal_id
// Returns true if path found, and fills path_ids with node ids (start..goal).
// For non-weighted algorithms, weight is ignored. For WASTAR, weight > 1.0.
bool run_search(
    const Graph &g,
    int start_id,
    int goal_id,
    SearchType type,
    std::vector<int> &path_ids,
    double weight = 2.0);
