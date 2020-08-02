#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "CommonVisualizer.hpp"

int main(int argn, char** argv) {
  using namespace std::string_literals;

  loco::CommonVisualizer visualizer(800, 1200, "armadillo", loco::color::BLACK);
  if (!visualizer.isInited()) {
    std::cout << "initialization failed" << std::endl;
    return 1;
  }
  visualizer.setLightSource(loco::Vec{0.0f, 0.0f, 10.0f}, loco::color::WHITE);
  visualizer.setDistance(5.0f);
  visualizer.setTransformCamera(loco::Transform{
      loco::Vec{0.0f, 0.0f, 0.0f}, loco::Vec{0.0f, 0.0f, 0.0f, 1.0f}});
  visualizer.setTheta(M_PI / 2.0);

  const std::string filename{argv[1]};

  const YAML::Node root{YAML::LoadFile(filename)};

  std::unordered_map<std::size_t, std::vector<float>> landmarks;
  std::vector<float> pos_mark;
  pos_mark.reserve(root["mark"].size() * 3u);
  for (const auto& mark : root["mark"]) {
    const std::vector<float> pos{mark.second["pos"].as<std::vector<float>>()};
    landmarks[mark.first.as<std::size_t>()] = pos;
    pos_mark.push_back(pos[0]);
    pos_mark.push_back(pos[1]);
    pos_mark.push_back(pos[2]);
  }

  std::unordered_map<std::size_t, std::vector<float>> cameras;
  std::vector<float> pos_cam;
  std::map<std::string, std::pair<std::size_t, std::vector<std::size_t>>>
      frames;
  pos_cam.reserve(root["key_frame"].size() * 3u);
  for (const auto& kf : root["key_frame"]) {
    const std::size_t id{kf.first.as<std::size_t>()};
    std::vector<float> pos{kf.second["tf"].as<std::vector<float>>()};
    pos.resize(3u);
    cameras[id] = pos;

    pos_cam.push_back(pos[0]);
    pos_cam.push_back(pos[1]);
    pos_cam.push_back(pos[2]);

    std::vector<std::size_t> has;
    has.reserve(kf.second["has"].size());
    for (const auto& h : kf.second["has"]) {
      has.push_back(h.first.as<std::size_t>());
    }
    frames[kf.second["name"].as<std::string>()] =
        std::pair<std::size_t, std::vector<std::size_t>>{id, has};
  }

  loco::Vec landmark_color{0.0f, 1.0f, 0.0f, 0.3f};
  loco::Vec keyframe_color{1.0f, 0.0f, 0.0f, 0.7f};
  visualizer.addPointCloud(pos_mark, landmark_color, 0.01f);
  visualizer.addPointCloud(pos_cam, keyframe_color, 0.02f);
  const std::string vis_name_links{"visible"};
  visualizer.setActiveWorld(vis_name_links);

  while (true) {
    for (const auto& frame : frames) {
      const std::size_t num_p{frame.second.second.size()};
      const std::vector<loco::Vec> colors(num_p, loco::color::BLUE);
      const std::vector<float> widths(num_p, 0.003f);
      std::vector<float> ends;
      ends.reserve(6u * num_p);
      const std::vector<float> pos_frame{cameras[frame.second.first]};
      for (const std::size_t id_p : frame.second.second) {
        const std::vector<float> pos_mark{landmarks[id_p]};
        ends.push_back(pos_frame[0]);
        ends.push_back(pos_frame[1]);
        ends.push_back(pos_frame[2]);
        ends.push_back(pos_mark[0]);
        ends.push_back(pos_mark[1]);
        ends.push_back(pos_mark[2]);
      }
      visualizer.addCylinder(ends, widths, colors);

      if (!visualizer.playOnce()) {
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      visualizer.resetScene(vis_name_links);
    }
    for (std::size_t i{0}; i < 10u; ++i) {
      if (!visualizer.playOnce()) {
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
}