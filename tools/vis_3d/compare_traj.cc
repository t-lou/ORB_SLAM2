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

  const std::vector<float> gt_pos{LoadCsv(std::string{argv[1]}, ',', 30u)};
  const std::vector<float> traj1_pos{LoadCsv(std::string{argv[2]}, ' ', 1u)};
  const std::vector<float> traj2_pos{LoadCsv(std::string{argv[3]}, ' ', 1u)};

  visualizer.addLine(gt_pos, 0.02f, loco::color::RED);
  visualizer.addLine(traj1_pos, 0.02f, loco::color::YELLOW);
  visualizer.addLine(traj2_pos, 0.02f, loco::color::GREEN);

  while (visualizer.playOnce()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}