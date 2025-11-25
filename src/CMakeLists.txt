# /home/changmin/PyProject/planning-example/src/CMakeLists.txt

# Put all executables into build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Use src as a global include root
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ---------- env library ----------
add_library(env STATIC
    env/environment.cc
    env/obstacle.cc
)

target_include_directories(env
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/env
)

# ---------- roadmap library ----------
add_library(roadmap STATIC
    roadmap/prm.cc
    roadmap/visibility.cc
    roadmap/gvd.cc
    roadmap/rrt.cc
    roadmap/sampling.cpp
)

target_include_directories(roadmap
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/roadmap
)

target_link_libraries(roadmap
    PUBLIC
        env
)

# ---------- build_env executable ----------
add_executable(build_env
    env/build_env.cc
)

target_link_libraries(build_env
    PRIVATE
        env
)

# ---------- build_roadmap executable ----------
add_executable(build_roadmap
    roadmap/build_roadmap.cc
)

target_link_libraries(build_roadmap
    PRIVATE
        env
        roadmap
)
