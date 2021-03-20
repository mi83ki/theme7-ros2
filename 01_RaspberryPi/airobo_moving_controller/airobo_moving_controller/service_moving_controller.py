from airobo_interfaces.srv import AiroboGoalPose

import rclpy
from rclpy.node import Node


class MovingControllerService(Node):

    def __init__(self):
        super().__init__('move_service')
        self.srv = self.create_service(AiroboGoalPose, 'move_service', self.moving_controll_callback)

    def moving_controll_callback(self, request, response):
        self.get_logger().info('Incoming request\nx: %f y: %f theta: %f' % (request.pose2d.x, request.pose2d.y, request.pose2d.theta))
        response.result = AiroboGoalPose.SUCCESS
        return response


def main(args=None):
    rclpy.init(args=args)

    move_service = MovingControllerService()

    rclpy.spin(moving_controller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()