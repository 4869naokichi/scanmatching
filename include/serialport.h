/**
 * @file serialport.cpp
 * @brief Windows APIを用いてシリアル通信を行う
 * @author naokichi
 * @date 2023/02/27
 */

#pragma once

#include <windows.h>

#include <string>

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief シリアル通信を行うクラス
 */
class SerialPort {
 private:
  /** COMポートのハンドラ */
  HANDLE hComPort;
  /** シリアルポート名 */
  std::string port;
  /** ボーレート */
  DWORD baudrate;

 public:
  /**
   * @brief COMポートをセットアップする
   * @retval TRUE 成功
   * @retval FALSE 失敗
   */
  BOOL SetupComPort(std::string port, DWORD baudrate);

  /**
   * @brief 1バイトのデータを送信する
   * @param txData 送信するデータ
   * @retval TRUE 送信成功
   * @retval FALSE 送信失敗
   */
  BOOL WriteByte(uint8_t txData);

  /**
   * @brief 1バイトのデータを受信する
   * @return 受信したデータ
   * @note ポーリングで受信する
   */
  uint8_t ReadByte();

  /**
   * @brief 文字列を送信する
   * @param txData 送信する文字列
   * @retval TRUE 送信成功
   * @retval FALSE 送信失敗
   */
  BOOL WriteLine(const std::string txData);

  /**
   * @brief 文字列を受信する
   * @return 受信した文字列
   * @note ポーリングで受信する
   */
  std::string ReadLine();

  /**
   * @brief float型の数値を送信する
   * @param f 送信する数値
   * @retval TRUE 送信成功
   * @retval FALSE 送信失敗
   */
  BOOL WriteFloat(float f);
};

}  // namespace my_lib