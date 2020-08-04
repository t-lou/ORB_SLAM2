#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class Viewer {
 public:
  Viewer(const std::int32_t width, const std::int32_t height)
      : _width(width), _height(height) {
    glEnable(GL_DEPTH_TEST);
    _d_cam.SetDrawFunction([&](pangolin::View& view) {
      view.Activate(_s_cam);
      _tree.Render();
    });
  }

  void LoadYaml(const std::string& filename) {
    _landmarks.clear();
    _cameras.clear();
    _frames.clear();

    const YAML::Node root{YAML::LoadFile(filename)};

    for (const auto& mark : root["mark"]) {
      const std::vector<float> pos{mark.second["pos"].as<std::vector<float>>()};
      _landmarks[mark.first.as<std::size_t>()] = pos;
    }

    for (const auto& kf : root["key_frame"]) {
      const std::size_t id{kf.first.as<std::size_t>()};
      std::vector<float> pos{kf.second["tf"].as<std::vector<float>>()};
      pos.resize(3u);
      _cameras[id] = pos;

      std::vector<std::size_t> has;
      has.reserve(kf.second["has"].size());
      for (const auto& h : kf.second["has"]) {
        has.push_back(h.first.as<std::size_t>());
      }
      _frames[kf.second["name"].as<std::string>()] =
          std::pair<std::size_t, std::vector<std::size_t>>{id, has};
      _frame_name.push_back(kf.second["name"].as<std::string>());
    }

    std::sort(_frame_name.begin(), _frame_name.end());
    _index = 0u;
  }

  void DisplayNext() {
    std::size_t frame_id;
    std::vector<std::size_t> frame_contents;
    std::tie(frame_id, frame_contents) = _frames[_frame_name[_index]];
    _index = (_index + 1u) % _frame_name.size();

    glPointSize(3);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 0.0f);
    for (std::size_t lm_id : frame_contents) {
      const std::vector<float> pos = _landmarks.at(lm_id);
      glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();

    glLineWidth(0.3);
    glBegin(GL_LINES);
    glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
    const std::vector<float> ori = _cameras.at(frame_id);
    for (std::size_t lm_id : frame_contents) {
      const std::vector<float> pos = _landmarks.at(lm_id);
      glVertex3f(pos[0], pos[1], pos[2]);
      glVertex3f(ori[0], ori[1], ori[2]);
    }
    glEnd();
  }

 private:
  const std::int32_t _width;

  const std::int32_t _height;

  pangolin::OpenGlRenderState _s_cam{[=]() {
    pangolin::CreateWindowAndBind("Main", _width, _height);
    return pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(_width, _height, _height / 2, _height / 2,
                                   _width / 2, _height / 2, 0.2, 100),
        pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));
  }()};

  pangolin::Renderable _tree{[]() {
    pangolin::Renderable tree;
    tree.Add(std::make_shared<pangolin::Axis>());
    return tree;
  }()};

  pangolin::SceneHandler _handler{_tree, _s_cam};

  pangolin::View& _d_cam{
      pangolin::CreateDisplay()
          .SetBounds(0.0f, 1.0f, 0.0f, 1.0f, -(float)_width / (float)_height)
          .SetHandler(&_handler)};

  std::unordered_map<std::size_t, std::vector<float>> _landmarks;
  std::unordered_map<std::size_t, std::vector<float>> _cameras;
  std::unordered_map<std::string,
                     std::pair<std::size_t, std::vector<std::size_t>>>
      _frames;
  std::size_t _index;
  std::vector<std::string> _frame_name;
};

int main(int argc, char** argv) {
  Viewer viewer(800, 600);
  std::deque<std::vector<float>> points;

  const std::string filename{argv[1]};

  viewer.LoadYaml(filename);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    viewer.DisplayNext();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
