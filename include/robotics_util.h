/**
 * @file robotics_util.h
 * @brief あ
 * @author naokichi
 * @date 2023/03/05
 */

#pragma once

#include <vector>

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief 2次元点群を格納する構造体
 * @note 直交座標で表現します
 */
struct PointCloud {
  std::vector<float> x;
  std::vector<float> y;
};

/**
 * @brief 姿勢を格納する構造体
 */
struct Pose {
  float x;
  float y;
  float theta;
};

}