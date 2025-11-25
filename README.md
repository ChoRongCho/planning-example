# README.md

## Planning Example Project

This project provides a modular pipeline for generating 2D planning environments, building roadmaps, running search algorithms, and visualizing the results.
All components are isolated for clarity and reproducibility:

* **C++** handles environment generation, roadmap construction, and path search.
* **Python** orchestrates these modules via `main.py` and visualizes outputs.

---

## 1. Installation

Clone the repository and install Python requirements:

```bash
git clone <this-repo>
cd planning-example

pip install -r requirements.txt
```

> Python is only used for orchestration and visualization.
> All planning modules are written in C++ and built through CMake.

---

## 2. Build Instructions

The project provides a build helper:

```bash
./build.py         # Configure + build (default: Release)
./build.py --debug # Build in Debug mode
./build.py --clean # Remove build directory
```

After building, all binaries appear in:

```
build/bin/
    ├── build_env
    ├── build_roadmap     (future)
    └── build_path        (future)
```

Only `build_env` is implemented at this stage.

---

## 3. build_env: Role and Specifications

`build_env` is responsible for generating a **random 2D continuous environment** with polygonal obstacles and saving it to:

```
results/env.txt
```

### Usage

```bash
./build/bin/build_env <num_obstacles> [seed]
```

* `<num_obstacles>` : requested number of obstacles
* `[seed]` (optional) : fixed seed for reproducible environments
* If seed is omitted, a random device generates one
* The actual seed is always written to `env.txt`

Python wrapper:

```bash
python main.py --env
python main.py --env --num_obstacles 12 --seed 123
```

---

## Environment Specification (Implemented Constraints)

### World

* Continuous 2D space: **20 × 20**

### Start & Goal

* Start: **(0.5, 0.5)**
* Goal: **(19.5, 19.5)**
* These points **must not lie inside any obstacle**

### Obstacle Count

* Requested M is clamped:

  * **0 ≤ M ≤ 20**

### Obstacle Polygon Properties

Each obstacle is a **simple Jordan curve polygon** with:

* **Vertex count:** 4 to 8 (uniform random)
* **Area:** between **5 and 18** units² (shoelace formula)
* **Sampled uniformly** inside the world boundary
* **No overlap allowed**, ensured by:

  * Edge–edge intersection checks
  * Vertex-in-polygon checks (both directions)

### Output Format (env.txt)

```
# seed <integer>
<M>                # number of generated obstacles
<K1>               # vertices in obstacle 1
x y
x y
...
<K2>
x y
...
<start_x> <start_y>
<goal_x>  <goal_y>
```

This file is later consumed by roadmap and search modules.

---

## Running From Python

Environment generation:

```bash
python main.py --env
```

Environment + automatic PNG/GIF save:

```bash
python main.py --env --save
```

Visualizer outputs PNG and GIF snapshots in `results/`.
