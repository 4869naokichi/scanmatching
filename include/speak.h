/**
 * @file speak.h
 * @brief 音声を喋らせる
 * @author naokichi
 * @date 2023/02/27
 * @note 実行ファイルと同じディレクトリにArgumentSpeak.vbsを配置してください
 */

#pragma once

#include <string>

/**
 * @brief 自作ライブラリ用の名前空間
 */
namespace my_lib {

/**
 * @brief メッセージを喋らせる
 * @param msg 喋らせるメッセージの文字列
 */
void Speak(const std::string msg);

} // namespace my_lib