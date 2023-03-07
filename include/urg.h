/**
 * @file urg.h
 * @brief LRFと通信し、データを読み取る
 * @author naokichi
 * @date 2023/02/27
 */

#pragma once

#define _USE_MATH_DEFINES

#include "robotics_util.h"
#include "serialport.h"

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief URG
 */
class URG {
 private:
  SerialPort serial;

 public:
  /**
   * @brief LRFをセットアップする
   * @param port シリアルポート名
   * @param baudrate ボーレート
   * @retval TRUE 成功
   * @retval FALSE 失敗
   */
  BOOL Setup(std::string port, DWORD baudrate);

  /**
   * @brief LRFからの電文を受信する
   * @return 受信した電文
   * @note 終端コードはLFLF
   */
  std::string ReadLine();

  /**
   * @brief レーザースキャンデータを取得する
   * @return レーザースキャンデータ（直交座標）
   */
  PointCloud GetScan();

  /**
   * @brief LRFから送信されたデータをデコードする
   * @param rawData エンコードされたデータ
   * @return 距離データ
   */
  std::vector<int> Decode(std::string rawData);

  /**
   * @brief LRFを停止する
   * @retval TRUE 成功
   * @retval FALSE 失敗
   */
  BOOL Stop();
};
}  // namespace my_lib