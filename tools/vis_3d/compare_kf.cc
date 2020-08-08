#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "CommonVisualizer.hpp"

auto LoadCsv(const std::string& fn, const char sep, const std::size_t interval)
    -> std::vector<float> {
  if (!fn.empty() and
      std::filesystem::is_regular_file(std::filesystem::path{fn})) {
    std::vector<float> ret;
    std::ifstream infile{fn};
    std::string line;
    std::size_t c{0u};
    while (std::getline(infile, line)) {
      if (line.size() > 1u and line[0] != '#') {
        std::string part;
        std::istringstream is{line};
        std::vector<float> parts;
        while (std::getline(is, part, sep) and !part.empty() and
               parts.size() < 4u) {
          parts.push_back(std::stof(part));
        }
        if (parts.size() >= 4u and (c++) % interval == 0u) {
          ret.push_back(parts[1]);
          ret.push_back(parts[2]);
          ret.push_back(parts[3]);
        }
      }
    }
    return ret;
  }
  return std::vector<float>{};
}

int main(int argn, char** argv) {
  using namespace std::string_literals;

  loco::CommonVisualizer visualizer(800, 1200, "armadillo", loco::color::BLACK);
  if (!visualizer.isInited()) {
    std::cout << "initialization failed" << std::endl;
    return 1;
  }
  visualizer.setLightSource(loco::Vec{0.0f, 0.0f, 10.0f}, loco::color::WHITE);
  visualizer.setDistance(15.0f);
  visualizer.setTransformCamera(loco::Transform{
      loco::Vec{0.0f, 0.0f, 0.0f}, loco::Vec{0.0f, 0.0f, 0.0f, 1.0f}});
  visualizer.setTheta(M_PI / 2.0);

  const std::string filename{argv[1]};

  const std::vector<float> gt_pos{
      LoadCsv(std::string{argn > 2 ? argv[2] : ""}, ',', 30u)};
  const std::vector<float> traj_pos{
      LoadCsv(std::string{argn > 3 ? argv[3] : ""}, ' ', 1u)};

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
  std::vector<float> cam_links;
  std::map<std::string, std::pair<std::size_t, std::vector<std::size_t>>>
      frames;
  for (const auto& kf : root["key_frame"]) {
    const std::size_t id{kf.first.as<std::size_t>()};
    std::vector<float> pos{kf.second["tf"].as<std::vector<float>>()};
    pos.resize(3u);
    cameras[id] = pos;

    std::vector<std::size_t> has;
    has.reserve(kf.second["has"].size());
    for (const auto& h : kf.second["has"]) {
      has.push_back(h.first.as<std::size_t>());
    }
    frames[kf.second["name"].as<std::string>()] =
        std::pair<std::size_t, std::vector<std::size_t>>{id, has};
  }
  pos_cam.reserve(frames.size() * 3u);
  for (const auto& kf : frames) {
    const auto& pos{cameras[kf.second.first]};
    pos_cam.insert(pos_cam.end(), pos.begin(), pos.end());
  }

  visualizer.addLine(pos_cam, 0.02f, loco::color::BLUE);
  if (!gt_pos.empty()) {
    visualizer.addLine(gt_pos, 0.02f, loco::color::RED);
  }
  if (!traj_pos.empty()) {
    visualizer.addLine(traj_pos, 0.02f, loco::color::YELLOW);
  }

  while (visualizer.playOnce()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}