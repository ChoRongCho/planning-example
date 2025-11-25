// /home/changmin/PyProject/planning-example/src/env/rng.h
#pragma once
#include <random>

class RNG {
public:
    static std::mt19937& get() {
        static std::mt19937 rng_instance;
        return rng_instance;
    }

    static void seed(unsigned int s) {
        get().seed(s);
    }

    static double uniform(double a = 0.0, double b = 1.0) {
        std::uniform_real_distribution<double> dist(a, b);
        return dist(get());
    }

    static int uniformInt(int a, int b) {
        std::uniform_int_distribution<int> dist(a, b); // inclusive
        return dist(get());
    }
};
