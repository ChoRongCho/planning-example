// /src/roadmap/visibility.h
#pragma once
#include "roadmap/graph.h"
#include "env/environment.h"
#include <ostream>

Graph buildVisibilityGraph(const Environment &env,
                           std::ostream *log = nullptr);
