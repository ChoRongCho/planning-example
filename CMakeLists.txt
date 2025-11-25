cmake_minimum_required(VERSION 3.10)

project(planning-example LANGUAGES CXX)

# 기본 C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 빌드 타입 기본값 (없으면 Release로)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# src 디렉토리로 위임
add_subdirectory(src)
