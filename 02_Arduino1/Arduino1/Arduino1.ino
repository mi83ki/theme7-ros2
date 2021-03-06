/***********************************************************************/
/*                                                                     */
/*  FILE        :Arduino1.ino                                          */
/*  DATE        :May 10, 2020                                          */
/*  DESCRIPTION :ROSロボ Arduino1用プログラム                          */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/
#include "fix.hpp"
#include "CTimer.hpp"
#include "CMotor.hpp"
#include "CBumper.hpp"
#include "CComArduinos.hpp"
#include "SpeedController.hpp"
#include "COdometry.hpp"
#include "CAccelController.hpp"
#include "CComRos2Arduino.hpp"
#include "CLog.hpp"

/***********************************************************************/
/*                           グローバル変数                            */
/***********************************************************************/


/***********************************************************************/
/*                               main関数                              */
/***********************************************************************/
void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // モータークラス
  static CMotor cMotor;
  // バンパークラス
  static CBumper cBumper;
  // タイマークラス
  static CTimer bpTimer(BUMPER_CYCLE_TIME); // バンパー用
  static CTimer spTimer(PID_CYCLE_TIME);    // 速度制御用
  static CTimer rosTimer(ROS_CYCLE_TIME);  // ros2arduino用
  // Arduino間通信クラス
  static CComArduinos cComArd(SLAVE);
  //速度制御インスタンス生成
  static SpeedController cSpCtrlR;
  static SpeedController cSpCtrlL;
  // 加速制御
  static CAccelController cAccCtrlR(PID_CYCLE_TIME);
  static CAccelController cAccCtrlL(PID_CYCLE_TIME);
  // ROS2Arduino通信クラス
  static CComRos2Arduino cRos2Ard;

  static uint8_t encUpdFlag = 0;

  // エンコーダ値更新処理
  if (cComArd.isI2Crecieved()) {
    cComArd.i2cSlaveRecieve();
    #ifdef ENCODER_REVERSE_RIGHT
    cComArd.A2state.encR *= -1;
    #endif
    #ifdef ENCODER_REVERSE_LEFT
    cComArd.A2state.encL *= -1;
    #endif
    encUpdFlag++;
  }

  // バンパーのサンプリング(CBumper.hppのBUMPER_SAMPLE_FREQ [Hz]で実行)
  if (bpTimer.isCycleTime()) {
    cBumper.bumperSampling();
  }

  // 速度制御
  if (spTimer.isCycleTime()) {
    //現在速度の計算
    cSpCtrlR.calcVelocity(cComArd.A2state.encR, cComArd.A2state.time);
    cSpCtrlL.calcVelocity(cComArd.A2state.encL, cComArd.A2state.time);
    //加速制御
    cAccCtrlR.accelSpeed();
    cAccCtrlL.accelSpeed();
    //目標速度の設定
    cSpCtrlR.setTargetVelocity(cAccCtrlR.getPresentVelocity());
    cSpCtrlL.setTargetVelocity(cAccCtrlL.getPresentVelocity());
    //PID制御
    cSpCtrlR.controlMotorsSpeed();
    cSpCtrlL.controlMotorsSpeed();
    //モーターにデューティ比を指令
    cMotor.driveMotors(cSpCtrlR.getDuty(), cSpCtrlL.getDuty());
  }

  // ROSへの周期的なパブリッシュ
  if (rosTimer.isCycleTime()) {
    // 状態通知パラメータの更新
    static uint32_t lastTime;
    static int32_t lastEncR;
    static int32_t lastEncL;
    if (lastTime) {
      cRos2Ard.updateArd2RosData(
        cComArd.A2state.time - lastTime,
        cComArd.A2state.encR - lastEncR,
        cComArd.A2state.encL - lastEncL,
        cBumper.readBumper()
      );
    }
    lastTime = cComArd.A2state.time;
    lastEncR = cComArd.A2state.encR;
    lastEncL = cComArd.A2state.encL;
    // 状態通知の送信
    cRos2Ard.txProcess();
  }

  // ROSからの速度変更指示チェック
  if (cRos2Ard.rxProcess()) {
    // 速度変更指示を受信したら、目標速度を更新する
    cAccCtrlR.setTargetVelocity(
      COdometry::twist2velocityR(
        FLOAT_TO_FIX(cRos2Ard.getVelocityX()),
        FLOAT_TO_FIX(cRos2Ard.getOmegaZ())
      )
    );
    cAccCtrlL.setTargetVelocity(
      COdometry::twist2velocityL(
        FLOAT_TO_FIX(cRos2Ard.getVelocityX()),
        FLOAT_TO_FIX(cRos2Ard.getOmegaZ())
      )
    );
  }
}
