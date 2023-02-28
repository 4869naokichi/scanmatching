/**
 * @file scanmatching.cpp
 * @brief スキャンマッチングを行う
 * @author naokichi
 * @date 2023/02/28
 */

#include <scanmatching.h>

#include <Eigen/Core>

/**
 * @brief ICPスキャンマッチングを行う
 * @param laser レーザースキャンの点群
 * @param map 地図の点群
 * @param pose あ
 * @return 姿勢の推定値
 */
std::tuple<float, float, float> MatchScansICP(
    std::tuple<std::vector<float>, std::vector<float>> laser,
    std::tuple<std::vector<float>, std::vector<float>> map,
    std::tuple<float, float, float> pose) {
  constexpr int kMaxIterations = 400;
  constexpr int kValidDistance = 400;  // [mm]
  constexpr float kScoreTolerance = 1e-6;

  auto [x, y, theta] = pose;
  Eigen::MatrixXf t(2, 1);
  t << x, y;
  Eigen::Matrix2f R;
  R << cos(theta), -sin(theta), sin(theta), cos(theta);
  // laser = R * laser + t;

  float E_prev = 0;
  for (int i = 0; i < kMaxIterations; i++) {
    ;
  }
}