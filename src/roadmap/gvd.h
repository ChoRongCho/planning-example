// /src/roadmap/gvd.h
#pragma once
#include <ostream>
#include "roadmap/graph.h"
#include "env/environment.h"

// jc_voronoi 기반 GVD Roadmap
Graph buildGVDVoronoi(const Environment &env,
                      std::ostream *out = nullptr);
