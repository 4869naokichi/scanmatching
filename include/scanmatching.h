/**
 * @file scanmatching.h
 * @brief スキャンマッチングを行う
 * @author naokichi
 * @date 2023/02/28
 */

#pragma once

#include <array>

#include "Eigen/Core"
#include "robotics_util.h"

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief kdtree.hを利用するために必要な構造体
 */
struct MyPoint : public std::array<float, 2> {
  static const int DIM = 2;
};

/**
 * @brief ICPスキャンマッチングを行う
 * @param pc_laser レーザースキャンの点群
 * @param pc_map 地図の点群
 * @param pose あ
 * @return 姿勢の推定値
 */
Pose MatchScansICP(PointCloud pc_laser, PointCloud pc_map, Pose pose);

/**
 * @brief PointCloud型からEigen行列への変換を行う
 * @param point_cloud PointCloud型のデータ
 * @return Eigen行列
 */
Eigen::MatrixXf PointCloudToMatrix(PointCloud point_cloud);

}  // namespace my_lib