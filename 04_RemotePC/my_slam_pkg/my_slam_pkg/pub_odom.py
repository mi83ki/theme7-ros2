 #!/usr/bin/env python  
import rclpy
from rclpy.node import Node


# Because of transformations
# import tf_conversions
# from tf_conversions.transformations import quaternion_from_euler
# from tf.transformations import *
import PyKDL
import tf2_ros
from geometry_msgs.msg import TransformStamped


from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry


class Odom_broadcaster(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.sub_enc = self.create_subscription(
            Int64MultiArray,
            'arduino/encoders',
            self.sub_enc_callback,
            10)
        self.sub_enc  # prevent unused variable warning

        self.pub_odom = self.create_publisher(
            Odometry,
            '/odom',
            10)

        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        x = 0.0
        y = 0.0
        th = 0.0
        WHEEL_BASE = 0.177
    
    def sub_enc_callback(self, msg):
        
        dt = msg.data[0]
        delta_enc_r = msg[1]
        delta_enc_l = msg[2]
        d_tire = 0.09
        gear_ratio = 150
        WHEEL_TREAD = 0.115

        #define TIRE_DIAMETER     (0.09f)    // タイヤの直径[m]
        #define WHEEL_TREAD       (0.115f)   // 車輪間距離[m]
        #define GEAR_RATIO        (150)      // ギア比
        # in COdometry.hpp & SpeedController.hpp

        #calculate tf from enc
        delta_r = delta_enc_r/3*gear_ratio*pi*d_tire
        delta_l = delta_enc_l/3*gear_ratio*pi*d_tire
        delta_p = (v_r + v_l) / 2.0
        delta_th = (delta_r - delta_l) / WHEEL_TREAD

        delta_x = delta_p * cos(th)
        delta_y = delta_p * sin(th)
        vx = delta_x/dt
        vy = delta_y/dt

        #tf2 update
        x = x + delta_x
        y = y + delta_y
        th = th + delta_th

        q = [0, 0, 0, 0]
        PyKDL.RPY(0,0,th).GetQuaternion(q[0],q[1],q[2],q[3])
        odom_quat = q
        # odom_quat = quaternion_from_euler(0, 0, th)
        current_time = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        odom_broadcaster.sendTransform(t)

        #topic update
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = current_time

        # set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        odom_pub.publish(odom)    


def main(args=None):
    rclpy.init(args=args)

    odom_broadcaster = Odom_broadcaster()

    rclpy.spin(odom_broadcaster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()