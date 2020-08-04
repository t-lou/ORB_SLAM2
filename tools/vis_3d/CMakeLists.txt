cmake_minimum_required(VERSION 3.6)
project(Examples)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(
  $ENV{HOME}/workspaces/CommonVisualizer/
)

add_executable(vis_3d main.cc)
target_link_libraries(vis_3d 
  GL GLEW GLU glut glfw
  yaml-cpp
)