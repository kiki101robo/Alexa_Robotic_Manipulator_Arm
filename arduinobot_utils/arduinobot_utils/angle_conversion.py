#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class AnglesConverter(Node):

    def __init__(self):
        super().__init__("angle_conversion_server")
        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulertoquaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaterniontoeulerCallback)
        self.get_logger().info("Angle_Conversion is Ready")



    def eulertoquaternionCallback(self, req, res):
        self.get_logger().info("New Request Received to convert roll: %f, pitch: %f, yaw: %f to quaternion" % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Returning respective quaternion: %f, %f, %f, %f" %(res.x, res.y, res.z, res.w))
        return res
    def quaterniontoeulerCallback(self, req, res):
        self.get_logger().info("New Request Received to convert x: %f, y: %f, z: %f, w: %f to euler" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion(req.x, req.y, req.z, req.w)
        self.get_logger().info("Returning respective eulers: %f, %f, %f" %(res.roll, res.pitch, res.yaw))
        return res


def main():
    rclpy.init()

    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    
    angles_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()