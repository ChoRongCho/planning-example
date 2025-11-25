// /src/search/search.cc
#include "search/search.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using std::vector;

SearchType parse_search_type(const std::string &name) {
    if (name == "bfs")
        return SearchType::BFS;
    if (name == "dfs")
        return SearchType::DFS;
    if (name == "gbfs")
        return SearchType::GBFS;
    if (name == "astar")
        return SearchType::ASTAR;
    if (name == "wastar")
        return SearchType::WASTAR;
    throw std::runtime_error("Unknown search type: " + name);
}

// 간단한 adjacency list 구성
static void build_adjacency(
    const Graph &g,
    vector<vector<std::pair<int, double>>> &adj)
{
    const int n = static_cast<int>(g.nodes.size());
    adj.assign(n, {});
    for (const auto &e : g.edges) {
        if (e.u < 0 || e.v < 0 || e.u >= n || e.v >= n)
            continue;
        adj[e.u].push_back({e.v, e.w});
        // 그래프를 무방향으로 쓸 생각이면 아래 줄도 추가
        // adj[e.v].push_back({e.u, e.w});
    }
}

// 공통: 경로 재구성
static bool reconstruct_path(
    int start_id,
    int goal_id,
    const vector<int> &parent,
    vector<int> &out_path)
{
    if (goal_id < 0 || goal_id >= static_cast<int>(parent.size()))
        return false;
    if (start_id < 0 || start_id >= static_cast<int>(parent.size()))
        return false;

    if (start_id == goal_id) {
        out_path.clear();
        out_path.push_back(start_id);
        return true;
    }

    if (parent[goal_id] == -1)
        return false;

    vector<int> rev;
    int cur = goal_id;
    while (cur != -1) {
        rev.push_back(cur);
        if (cur == start_id)
            break;
        cur = parent[cur];
    }
    if (rev.back() != start_id)
        return false;

    out_path.assign(rev.rbegin(), rev.rend());
    return true;
}

// ===== Uninformed: BFS / DFS =====

static bool bfs_search(
    const Graph &g,
    int start_id,
    int goal_id,
    vector<int> &path_ids)
{
    vector<vector<std::pair<int, double>>> adj;
    build_adjacency(g, adj);
    const int n = static_cast<int>(adj.size());
    if (start_id < 0 || start_id >= n || goal_id < 0 || goal_id >= n)
        return false;

    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    std::queue<int> q;

    visited[start_id] = true;
    q.push(start_id);

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        if (u == goal_id)
            break;

        for (auto &pr : adj[u]) {
            int v = pr.first;
            if (!visited[v]) {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }
    return reconstruct_path(start_id, goal_id, parent, path_ids);
}

static bool dfs_search(
    const Graph &g,
    int start_id,
    int goal_id,
    vector<int> &path_ids)
{
    vector<vector<std::pair<int, double>>> adj;
    build_adjacency(g, adj);
    const int n = static_cast<int>(adj.size());
    if (start_id < 0 || start_id >= n || goal_id < 0 || goal_id >= n)
        return false;

    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    std::stack<int> st;

    st.push(start_id);
    visited[start_id] = true;

    while (!st.empty()) {
        int u = st.top();
        st.pop();
        if (u == goal_id)
            break;
        for (auto &pr : adj[u]) {
            int v = pr.first;
            if (!visited[v]) {
                visited[v] = true;
                parent[v] = u;
                st.push(v);
            }
        }
    }

    return reconstruct_path(start_id, goal_id, parent, path_ids);
}

// ===== Informed: GBFS / A* / Weighted A* =====

struct PQNode {
    int id;
    double g;
    double f;
};

struct PQCompare {
    bool operator()(const PQNode &a, const PQNode &b) const {
        return a.f > b.f; // 작은 f가 먼저 나오도록
    }
};

static double heuristic_dist(const Graph &g, int v, int goal_id) {
    const Vec2 &p = g.nodes[v].p;
    const Vec2 &gpos = g.nodes[goal_id].p;
    double dx = p.x - gpos.x;
    double dy = p.y - gpos.y;
    return std::sqrt(dx * dx + dy * dy);
}

static bool best_first_search(
    const Graph &g,
    int start_id,
    int goal_id,
    SearchType type,
    vector<int> &path_ids,
    double weight)
{
    vector<vector<std::pair<int, double>>> adj;
    build_adjacency(g, adj);
    const int n = static_cast<int>(adj.size());
    if (start_id < 0 || start_id >= n || goal_id < 0 || goal_id >= n)
        return false;

    const double INF = std::numeric_limits<double>::infinity();
    vector<double> gval(n, INF);
    vector<int> parent(n, -1);
    vector<bool> closed(n, false);

    std::priority_queue<PQNode, vector<PQNode>, PQCompare> open;

    gval[start_id] = 0.0;
    double h0 = heuristic_dist(g, start_id, goal_id);
    double f0;
    switch (type) {
        case SearchType::GBFS:
            f0 = h0;
            break;
        case SearchType::ASTAR:
            f0 = gval[start_id] + h0;
            break;
        case SearchType::WASTAR:
            f0 = gval[start_id] + weight * h0;
            break;
        default:
            f0 = h0;
            break;
    }
    open.push({start_id, 0.0, f0});

    while (!open.empty()) {
        PQNode cur = open.top();
        open.pop();

        int u = cur.id;
        if (closed[u])
            continue;
        closed[u] = true;

        if (u == goal_id)
            break;

        for (auto &pr : adj[u]) {
            int v = pr.first;
            double w = pr.second;
            if (closed[v])
                continue;

            double g_new;
            if (type == SearchType::GBFS) {
                // GBFS는 g를 중요하게 쓰지 않지만
                // parent 관리를 위해 한 번만 방문
                g_new = 0.0;
                if (parent[v] != -1)
                    continue;
            } else {
                g_new = gval[u] + w;
                if (g_new >= gval[v])
                    continue;
            }

            parent[v] = u;
            gval[v] = g_new;

            double h = heuristic_dist(g, v, goal_id);
            double f;
            if (type == SearchType::GBFS)
                f = h;
            else if (type == SearchType::ASTAR)
                f = g_new + h;
            else
                f = g_new + weight * h; // WASTAR

            open.push({v, g_new, f});
        }
    }

    return reconstruct_path(start_id, goal_id, parent, path_ids);
}

bool run_search(
    const Graph &g,
    int start_id,
    int goal_id,
    SearchType type,
    vector<int> &path_ids,
    double weight)
{
    switch (type) {
    case SearchType::BFS:
        return bfs_search(g, start_id, goal_id, path_ids);
    case SearchType::DFS:
        return dfs_search(g, start_id, goal_id, path_ids);
    case SearchType::GBFS:
        return best_first_search(g, start_id, goal_id, type, path_ids, weight);
    case SearchType::ASTAR:
        return best_first_search(g, start_id, goal_id, type, path_ids, weight);
    case SearchType::WASTAR:
        if (weight <= 1.0)
            weight = 3;
        return best_first_search(g, start_id, goal_id, type, path_ids, weight);
    default:
        return false;
    }
}
