// /src/roadmap/sampling.h
#pragma once
#include <cstddef>
#include "env/environment.h"

double halton(int index, int base);

// Uniform random free-space sample
Vec2 sampleUniformFree(const Environment &env);

// 2D Halton-based low-discrepancy sample
Vec2 sampleHaltonFree(const Environment &env, std::size_t idx);

// Simple Sobol-like sample (using different Halton bases)
Vec2 sampleSobolFree(const Environment &env, std::size_t idx);
