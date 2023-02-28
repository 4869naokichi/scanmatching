/**
 * @file scanmatching.h
 * @brief スキャンマッチングを行う
 * @author naokichi
 * @date 2023/02/28
 */

#pragma once

#include <tuple>
#include <vector>

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
    std::tuple<float, float, float> pose);