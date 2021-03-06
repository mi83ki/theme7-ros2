/***********************************************************************/
/*                                                                     */
/*  FILE        :CEncoder.hpp                                          */
/*  DATE        :Jul 11, 2020                                          */
/*  DESCRIPTION :エンコーダカウントクラス                              */
/*                                                                     */
/*  This file is generated by Hideaki Shimada                          */
/*                            Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/

#ifndef _CENCODER_HPP__
#define _CENCODER_HPP__

#include <Arduino.h>

/***********************************************************************/
/*                             CountEncoder                            */
/*---------------------------------------------------------------------*/
/*                       エンコーダをカウントする                      */
/***********************************************************************/
/***************************   使い方の例   ****************************

// インスタンス生成
static CEncoder ge;

// 外部割込みINT6
ISR(INT6_vect) {
  ge::interruptEnc0A();
}

// ピン変化割込みPCINT0
ISR(PCINT0_vect) {
  ge::interruptEnc1B();
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(ge.getEncR());
  Serial.print(", ");
  Serial.println(ge.getEncL());
  delay(1000);
}

************************************************************************/


// エンコーダのピン
#define ENC0A 7        // PE6(INT6)
#define ENC0B 6        // PD7
#define ENC1A A0       // PF7
#define ENC1B 15       // PB1(PCINT1)

class CEncoder
{
public:
  CEncoder();
  // エンコーダーRの割込み
  // 外部割込みINT6
  void interruptEnc0A (void);
  // エンコーダーLの割込み
  // ピン変化割込みPCINT0
  void interruptEnc1B (void);
  int32_t getEncR(void);
  int32_t getEncL(void);
  void setEncR(int32_t eR);
  void setEncL(int32_t eL);

private:
  // エンコーダのカウント値
  int32_t encR;
  int32_t encL;
};

#endif  // _CENCODER_HPP__
