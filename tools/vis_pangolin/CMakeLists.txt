cmake_minimum_required(VERSION 3.6)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(vis_pangolin main.cc)
target_link_libraries(vis_pangolin
  ${Pangolin_LIBRARIES}
  yaml-cpp
)