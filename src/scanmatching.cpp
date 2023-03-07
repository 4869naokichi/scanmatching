/**
 * @file scanmatching.cpp
 * @brief スキャンマッチングを行う
 * @author naokichi
 * @date 2023/02/28
 */

#include <scanmatching.h>

#include <iostream>
#include <vector>

#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "kdtree.h"
#include "matplotlibcpp.h"
#include "timer.h"

namespace my_lib {

namespace plt = matplotlibcpp;

Pose MatchScansICP(PointCloud pc_laser, PointCloud pc_map, Pose pose) {
  /** 最大反復回数 */
  constexpr int kMaxIterations = 400;
  /** 有効とみなす距離 [mm] */
  constexpr int kValidDistance = 400;
  /** ICPスコアの変化に対する下限 */
  constexpr float kScoreTolerance = 1e-6;

  Eigen::MatrixXf laser = PointCloudToMatrix(pc_laser);
  Eigen::MatrixXf map = PointCloudToMatrix(pc_map);
  Eigen::Vector2f t;
  t << pose.x, pose.y;
  Eigen::Matrix2f R;
  R << cos(pose.theta), -sin(pose.theta), sin(pose.theta), cos(pose.theta);
  laser = (R * laser).colwise() + t;

  static float E_prev = 0;
  for (int i = 0; i < kMaxIterations; i++) {
    // 最近傍点を探索
    std::vector<MyPoint> map_points;
    for (int i = 0; i < map.cols(); i++) {
      MyPoint map_point = {map(0, i), map(1, i)};
      map_points.push_back(map_point);
    }
    kdt::KDTree<MyPoint> kdtree(map_points);
    std::vector<int> indices;
    std::vector<float> distances;
    for (int i = 0; i < laser.cols(); i++) {
      MyPoint laser_point = {laser(0, i), laser(1, i)};
      double distance;
      indices.push_back(kdtree.nnSearch(laser_point, &distance));
      distances.push_back(distance);
    }

    // プロット
    /*
    plt::clf();
    plt::scatter(pc_laser.x, pc_laser.y);
    plt::scatter(pc_map.x, pc_map.y);
    for (size_t i = 0; i < pc_laser.x.size(); i++) {
      std::vector<float> x = {pc_laser.x[i], pc_map.x[indices[i]]};
      std::vector<float> y = {pc_laser.y[i], pc_map.y[indices[i]]};
      plt::plot(x, y, "lime");
    }
    plt::xlim(-1000, 4000);
    plt::ylim(-1000, 4000);
    plt::set_aspect_equal();
    plt::pause(1e-6);
    */

    // 有効な点を抽出
    Eigen::MatrixXf laser_valid(2, 0);
    Eigen::MatrixXf map_valid(2, 0);
    for (int i = 0; i < laser.cols(); i++) {
      if (distances[i] < kValidDistance) {
        laser_valid.conservativeResize(Eigen::NoChange, laser_valid.cols() + 1);
        laser_valid.col(laser_valid.cols() - 1) = laser.col(i);
        map_valid.conservativeResize(Eigen::NoChange, map_valid.cols() + 1);
        map_valid.col(map_valid.cols() - 1) = map.col(indices[i]);
      }
    }

    // 誤差関数E(x)の値を計算
    float E = (laser_valid - map_valid).cwiseAbs2().sum() / laser_valid.cols();

    // 収束判定
    float delta_E = fabsf(E_prev - E);
    if (delta_E < kScoreTolerance) {
      break;
    }
    E_prev = E;

    // 最適化
    Eigen::Vector2f mu_laser = laser_valid.rowwise().mean();
    Eigen::MatrixXf laser_ = laser_valid.colwise() - mu_laser;
    Eigen::Vector2f mu_map = map_valid.rowwise().mean();
    Eigen::MatrixXf map_ = map_valid.colwise() - mu_map;

    Eigen::MatrixXf W = map_ * laser_.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        W, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix2f R1 = svd.matrixU() * svd.matrixV().transpose();
    Eigen::Vector2f t1 = mu_map - R1 * mu_laser;

    // 姿勢と点群を更新
    R = R1 * R;
    t = t + t1;
    laser = (R1 * laser).colwise() + t1;
  }
  pose.x = t(0);
  pose.y = t(1);
  Eigen::Matrix3f hoge = Eigen::MatrixXf::Identity(3, 3);
  hoge.topLeftCorner(2, 2) = R;
  Eigen::Vector3f euler = hoge.eulerAngles(0, 1, 2);
  pose.theta = euler(2);
  return pose;
}

Eigen::MatrixXf PointCloudToMatrix(PointCloud point_cloud) {
  size_t size = point_cloud.x.size();
  Eigen::Map<Eigen::RowVectorXf> x(point_cloud.x.data(), size);
  Eigen::Map<Eigen::RowVectorXf> y(point_cloud.y.data(), size);
  Eigen::MatrixXf matrix(2, size);
  matrix.row(0) = x;
  matrix.row(1) = y;
  return matrix;
}

}  // namespace my_lib