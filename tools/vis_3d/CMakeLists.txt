cmake_minimum_required(VERSION 3.6)
project(Examples)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
  $ENV{HOME}/workspaces/CommonVisualizer/
)

add_executable(vis_3d main.cc)
target_link_libraries(vis_3d 
  GL GLEW GLU glut glfw
  yaml-cpp
)

add_executable(vis_traj compare_traj.cc)
target_link_libraries(vis_traj 
  GL GLEW GLU glut glfw
)

add_executable(vis_kf compare_kf.cc)
target_link_libraries(vis_kf 
  GL GLEW GLU glut glfw
  yaml-cpp
)