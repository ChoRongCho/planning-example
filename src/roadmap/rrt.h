// /src/roadmap/rrt.h
#pragma once
#include "roadmap/graph.h"
#include "env/environment.h"
#include <ostream>  

Graph buildRRTGraph(const Environment &env,
                    int max_iter = 3000,
                    double step_size = 0.5,
                    double goal_threshold = 0.5,
                    double goal_bias = 0.05,
                    std::ostream *log = nullptr);