

int main(int argc, char** argv) {
    Environment env(seed);
    env.generate();

    Graph graph;

    if (roadmap == "prm") graph = PRM().build(env);
    else if (roadmap == "rrt") graph = RRT().build(env);
    ...

    auto path = AStar().search(graph);

    FileIO::save(env, graph, path);
}
