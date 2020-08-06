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

  loco::Vec landmark_color{0.0f, 1.0f, 0.0f, 0.3f};
  loco::Vec keyframe_color{1.0f, 0.0f, 0.0f, 0.7f};
  // visualizer.addPointCloud(pos_cam, keyframe_color, 0.02f);
  visualizer.addPointCloud(pos_mark, landmark_color, 0.01f);
  visualizer.addLine(pos_cam, 0.02f, keyframe_color);
  if (!gt_pos.empty()) {
    loco::Vec keyframe_gt{0.0f, 1.0f, 1.0f, 1.0f};
    visualizer.addLine(gt_pos, 0.02f, keyframe_gt);
  }
  if (!traj_pos.empty()) {
    loco::Vec keyframe_gt{1.0f, 0.0f, 1.0f, 1.0f};
    visualizer.addLine(traj_pos, 0.02f, keyframe_gt);
  }
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