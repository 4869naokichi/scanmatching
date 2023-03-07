/**
 * @file timer.h
 * @brief 処理時間を計測する
 * @author naokichi
 * @date 2023/02/27
 */

#pragma once

#include <windows.h>

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief 処理時間を計測するクラス
 */
class Timer {
private:
  LARGE_INTEGER freq;
  LARGE_INTEGER start, end;

public:
  /**
   * コンストラクタ
   */
  Timer();

  /**
   * @brief タイマーをリスタートする
   */
  void Restart();

  /**
   * @brief 経過時間を取得する
   * @return 経過時間 [s]
   */
  double GetElapsedTime();
};

} // namespace my_lib
