#include <conio.h>
#include <matplotlibcpp.h>
#include <scanmatching.h>
#include <speak.h>
#include <urg.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace plt = matplotlibcpp;

void Plot(my_lib::PointCloud scan, my_lib::PointCloud map, my_lib::Pose pose) {
  plt::clf();
  plt::scatter(scan.x, scan.y);
  plt::scatter(map.x, map.y);
  std::vector<float> x(1, pose.x);
  std::vector<float> y(1, pose.y);
  std::vector<float> u(1, cos(pose.theta));
  std::vector<float> w(1, sin(pose.theta));
  plt::quiver(x, y, u, w);
  plt::xlim(-1000, 4000);
  plt::ylim(-1000, 4000);
  plt::set_aspect_equal();
  plt::pause(1e-6);
}

my_lib::PointCloud LoadMap() {
  my_lib::PointCloud map;

  std::string line;
  std::string coordinate;
  std::ifstream data("csv/room.csv");

  for (int i = 0; getline(data, line); i++) {
    std::istringstream iss(line);
    while (std::getline(iss, coordinate, ',')) {
      if (i == 0)
        map.x.push_back(std::stof(coordinate));
      else if (i == 1)
        map.y.push_back(std::stof(coordinate));
    }
  }

  return map;
}

int main() {
  my_lib::URG urg;
  BOOL retval = urg.Setup("COM5", 115200);
  if (retval == FALSE) {
    return 0;
  }

  my_lib::Pose pose{0, 0, 0};
  my_lib::PointCloud map = LoadMap();

  while (TRUE) {
    if (_kbhit()) {
      if (getch() == 'q') break;
    }

    my_lib::PointCloud scan = urg.GetScan();
    pose = my_lib::MatchScansICP(scan, map, pose);
    // Plot(scan, map, pose);
    std::cout << pose.x << "\t" << pose.y << "\t" << pose.theta << std::endl;
  }

  urg.Stop();
}