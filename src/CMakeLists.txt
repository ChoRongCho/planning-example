# /home/changmin/PyProject/planning-example/src/CMakeLists.txt

# 모든 실행 파일을 build/bin 밑에 모으기
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# include 경로 설정 (src 전체를 include root로 사용)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ---------- env 라이브러리 ----------
add_library(env STATIC
    env/environment.cc
    env/obstacle.cc
)

target_include_directories(env
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/env
)

# ---------- build_env 실행파일 ----------
add_executable(build_env
    env/build_env.cc
)

target_link_libraries(build_env
    PRIVATE
        env
)

# ---------- (미래용) roadmap / search / utils / planner 뼈대 ----------
# 나중에 build_roadmap, build_path(또는 build_search) 만들 때 참고용 템플릿
#
# add_executable(build_roadmap
#     roadmap/build_roadmap.cc
# )
# target_link_libraries(build_roadmap
#     PRIVATE
#         env
#         roadmap
# )
#
# add_executable(build_path
#     search/build_path.cc
# )
# target_link_libraries(build_path
#     PRIVATE
#         env
#         roadmap
#         search
# )

# add_library(roadmap STATIC
#     roadmap/prm.cc
#     roadmap/rrt.cc
#     roadmap/visibility.cc
#     roadmap/gvd.cc
#     roadmap/sampling.cpp
# )
# target_include_directories(roadmap
#     PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/roadmap
# )

# add_library(search STATIC
#     search/astar.cpp
#     search/bfs.cpp
#     search/gbfs.cpp
#     search/search.cpp
# )
# target_include_directories(search
#     PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/search
# )

# add_library(utils STATIC
#     utils/geometry.cpp
#     utils/file_io.cpp
# )
# target_include_directories(utils
#     PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/utils
# )

# add_executable(planner
#     planner.cc
# )
# target_link_libraries(planner
#     PRIVATE
#         env
#         roadmap
#         search
#         utils
# )
