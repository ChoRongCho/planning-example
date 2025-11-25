// /src/roadmap/sampling.cpp
#include "roadmap/sampling.h"
#include "env/rng.h"
#include <cmath>

double halton(int index, int base) {
    double f = 1.0;
    double r = 0.0;
    int i = index;
    while (i > 0) {
        f /= base;
        r += f * (i % base);
        i /= base;
    }
    return r;
}

Vec2 sampleUniformFree(const Environment &env) {
    while (true) {
        double x = RNG::uniform(env.world_min, env.world_max);
        double y = RNG::uniform(env.world_min, env.world_max);
        Vec2 p(x, y);
        if (env.isFree(p))
            return p;
    }
}

Vec2 sampleHaltonFree(const Environment &env, std::size_t idx) {
    double hx = halton(static_cast<int>(idx), 2);
    double hy = halton(static_cast<int>(idx), 3);
    Vec2 p(env.world_min + (env.world_max - env.world_min) * hx,
           env.world_min + (env.world_max - env.world_min) * hy);
    if (!env.isFree(p))
        return sampleUniformFree(env);
    return p;
}

Vec2 sampleSobolFree(const Environment &env, std::size_t idx) {
    // very rough Sobol-like using different Halton bases
    double sx = halton(static_cast<int>(idx), 5);
    double sy = halton(static_cast<int>(idx), 7);
    Vec2 p(env.world_min + (env.world_max - env.world_min) * sx,
           env.world_min + (env.world_max - env.world_min) * sy);
    if (!env.isFree(p))
        return sampleUniformFree(env);
    return p;
}
