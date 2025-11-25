// /src/roadmap/prm.h
#pragma once
#include <string>
#include "roadmap/graph.h"
#include "env/environment.h"
#include <ostream>

enum class SamplerType {
    RANDOM,
    HALTON,
    SOBOL
};

SamplerType samplerFromString(const std::string &name);

// Build a PRM graph with N samples and connection radius

Graph buildPRM(const Environment &env, int n_samples,
               SamplerType sampler, double radius,
               std::ostream *log = nullptr);
