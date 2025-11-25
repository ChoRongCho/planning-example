// /src/roadmap/gvd.h
#pragma once
#include "roadmap/graph.h"
#include "env/environment.h"
#include <ostream>

Graph buildGVDLike(const Environment &env,
                   int grid_resolution = 40,
                   double band_min = 0.3,
                   double band_max = 1.5,
                   double connect_radius = 2.0,
                   std::ostream *log = nullptr);