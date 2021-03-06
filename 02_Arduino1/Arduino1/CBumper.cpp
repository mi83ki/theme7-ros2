/***********************************************************************/
/*                                                                     */
/*  FILE        :CBumper.cpp                                           */
/*  DATE        :Jun. 14, 2020                                         */
/*  DESCRIPTION :第二世代ROSロボ バンパーの接触検知モジュール          */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/

#ifndef _CBUMPER_CPP__
#define _CBUMPER_CPP__

#include "CBumper.hpp"

// コンストラクタ
CBumper::CBumper(void)
{
  pinMode(BUMPER_1, INPUT);

  // 一次のローパスフィルタの初期設定
  m_bpFilter = new CFirstFilter(HPF, CUT_OFF_FREQ, BUMPER_SAMPLE_FREQ);
}

// デストラクタ
CBumper::~CBumper(void)
{
  delete m_bpFilter;  // インスタンスの消去
}


// 現在のバンパー電圧[V]を返す
float CBumper::getPre(void)
{
  return FIX_TO_FLOAT(m_bpVoltage);
}

// バンパー電圧[V]のローパスフィルタ値を返す
float CBumper::getLPF(void)
{
  return FIX_TO_FLOAT(m_bpFilter->getLPF());
}

// バンパー電圧[V]のハイパスフィルタ値を返す
float CBumper::getHPF(void)
{
  return FIX_TO_FLOAT(m_bpFilter->getOUT());
}


// バンパー電圧[V]を返す
fix CBumper::readBumperVoltage(void)
{
  int sensorValue = analogRead(A1);
  fix voltage = INT_TO_FIX(sensorValue) * 5 / 1023;
  return(voltage);
}

// 一次のローパスフィルタにかける
void CBumper::filtBumperVoltage(fix pre)
{
  m_bpFilter->firstFiltering(pre);    // aをローパスフィルタm_bpFilterにかける
}


// サンプリング関数。SAMPLE_FREQで実行してあげる必要がある。
void CBumper::bumperSampling(void)
{
  filtBumperVoltage(readBumperVoltage());
}

// バンパーがONなら1、OFFなら0を返す
bool CBumper::readBumper(void)
{
  // 出力電圧比[%]を計算
  fix ratio = FIX_DIV(m_bpFilter->getOUT() * 100, m_bpFilter->getLPF());
  if (ratio < -1 * INT_TO_FIX(BUMPER_THRESHOLD)) {
    return true;
  } else {
    return false;
  }
}

// 引数のバンパー値を更新した上で、前回から変化したら1を、そうでなければ0を返す
bool CBumper::isBumperChanged(void)
{
  static uint8_t lastBumper;    // バンパーの前回値
  uint8_t preBumper = readBumper();

  if (preBumper != lastBumper) {
    lastBumper = preBumper;
    return (true);
  } else {
    return (false);
  }
}

#endif  // _BUMPER_CPP__
