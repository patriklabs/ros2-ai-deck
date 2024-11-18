import rclpy


import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, Image
from .ai_deck import AiDeck, IMU, ImageData, AiDeckCallbackInterface
import numpy as np


class AiDeckRelay(Node, AiDeckCallbackInterface):

    def __init__(self):
        super().__init__("AiDeckRelay")
        self.imu_publisher_ = self.create_publisher(Imu, "imu", 100)
        self.image_publisher_ = self.create_publisher(Image, "image", 100)

    def imu_callback(self, imu: IMU):
        imu_data = Imu()

        imu_data.angular_velocity.x = imu.gyro_x
        imu_data.angular_velocity.y = imu.gyro_y
        imu_data.angular_velocity.z = imu.gyro_z

        imu_data.linear_acceleration.x = imu.acc_x
        imu_data.linear_acceleration.y = imu.acc_y
        imu_data.linear_acceleration.z = imu.acc_z

        imu_data.header.stamp = Time(nanoseconds=imu.timestamp * 1e6).to_msg()
        self.imu_publisher_.publish(imu_data)

    def image_callback(self, image: ImageData):
        img_data = image.get_image()

        shape = list(np.shape(img_data))

        if len(shape) == 2:
            shape.append(1)

        msg = Image()
        msg.header.stamp = Time(nanoseconds=image.image_header.timestamp * 1e6).to_msg()
        msg.header.frame_id = "drone"
        msg.height = shape[0]
        msg.width = shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = shape[2] * shape[1]
        msg.data = np.array(img_data).tobytes()

        self.image_publisher_.publish(msg)

        # print("got image!")


def main(args=None):
    rclpy.init(args=args)

    relay = AiDeckRelay()
    ai_deck = AiDeck(show_image=True)

    while relay.context.ok():

        rclpy.spin_once(relay, timeout_sec=0)
        ai_deck.spin_once(relay)

    relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()