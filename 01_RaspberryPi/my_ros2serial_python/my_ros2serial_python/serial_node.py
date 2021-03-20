# -*- coding: utf-8 -*-

import serial
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

"""
シリアル通信クラス
"""
class SerialCom:
    # 初期化
    def __init__(self):
        # オープンフラグ
        self.isPortOpen = False
        # 受信データ
        self.recvData = ''
        # 受信フラグ
        self.recvFlag = False
        # 受信時のコールバック関数
        self.recvCallback = None

    # UART受信完了時のコールバック関数を登録する
    def setRecvCallback(self, func):
        self.recvCallback = func

    # データ受信チェック
    def isRecieved(self):
        if self.recvFlag:
            self.recvFlag = False
            return True
        else:
            return False

    # 受信データの取得
    def getRecvMsg(self):
        return self.recvData.decode()

    # データ受信待ち
    def recv(self):
        with self.comm:
            # データ受信待ち
            while True:
                # 受信データ読み取り
                buff = self.comm.readline()
                # 受信データ判定
                if len(buff) > 0:
                    #print(buff)
                    self.recvData = buff
                    self.recvFlag = True
                    # コールバック関数の実行
                    self.recvCallback(self.getRecvMsg())

    # データ送信
    def send(self, data):
        self.comm.write(data.encode())
        #print(data)

    # シリルポートオープン
    def open(self, tty, baud='115200'):
        try:
            self.comm = serial.Serial(tty, baud, timeout=0.1)
            self.isPortOpen = True
            # Thread起動 recvのやり取りをする
            t = threading.Thread(target = self.recv)
            t.setDaemon(True)
            t.start()
        except Exception as e:
            self.isPortOpen = False

        return self.isPortOpen


# ros2serial_pythonノードクラス
class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        # パブリッシャー
        self.pub_enc = self.create_publisher(Int64MultiArray, 'arduino/encoders', 10)
        print('create publisher: arduino/encoders')
        self.pub_bumper = self.create_publisher(Bool, 'arduino/bumper', 10)
        print('create publisher: arduino/bumper')
        # サブスクライバー
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'arduino/cmd_vel',
            self.cmdVelCallBack,
            10)
        self.sub_cmd_vel    # prevent unused variable warning
        print('create subscriber: arduino/cmd_vel')
        self.sub_cmd_vel2 = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVelCallBack,
            10)
        self.sub_cmd_vel2    # prevent unused variable warning
        self.get_logger().info('create subscriber: cmd_vel')
        # サブスクライブ時のコールバック関数
        self.subCallback = None
        # Arduino経過時間[ms]
        self.delta_t = 0
        # 右エンコーダ増分
        self.delta_enc_r = 0
        # 左エンコーダ増分
        self.delta_enc_l = 0
        # バンパー値
        self.bumper = 0
        self.bumper_last = 0

        # x方向速度[m/s]
        self.velocity_x = 0.0
        # z方向角速度[rad/s]
        self.omega_z = 0.0

    # サブスクライブ時のコールバック関数を登録する
    def setSubCallback(self, func):
        self.subCallback = func

    def decode(self, msg):
        data = msg.split(',')
        # 冒頭文字が正しい場合、値の更新
        if data[0] == 'a':
            self.delta_t = int(data[1])
            self.delta_enc_r = int(data[2])
            self.delta_enc_l = int(data[3])
            self.bumper = int(data[4])

            # エンコーダ値のパブリッシュ
            msg = Int64MultiArray()
            msg.data = [
                self.delta_t,
                self.delta_enc_r,
                self.delta_enc_l
            ]
            self.pub_enc.publish(msg)

            # バンパー値は変化のあるときだけパブリッシュする
            if self.bumper != self.bumper_last:
                self.bumper_last = self.bumper
                msg = Bool()
                msg.data = (self.bumper != 0)
                self.pub_bumper.publish(msg)

    def cmdVelCallBack(self, cmd_vel):
        self.velocity_x = cmd_vel.linear.x
        self.omega_z = cmd_vel.angular.z
        msg = 'r,'
        msg += '{:.2f}'.format(self.velocity_x)
        msg += ','
        msg += '{:.2f}'.format(self.omega_z)
        msg += ',\r\n'
        self.subCallback(msg)


# メイン関数
def main(args=None):
    rclpy.init(args=args)
    # ros2arduinoのメッセージ管理クラス
    serial_node = SerialNode()
    # シリアルを開く
    serial_com = SerialCom()
    serial_com.setRecvCallback(serial_node.decode)
    serial_node.setSubCallback(serial_com.send)
    serial_com.open('/dev/ttyAMA0', '115200')

    rclpy.spin(serial_node)

    # Destroy the node explicitly
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
