import time
from threading import Thread

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .motion_commander import MotionCommander


class MotionPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Twist, "setpoint", 10)

    def is_connected(self):
        return True

    def publish_motion(self, msg: Twist):
        self.publisher_.publish(msg)

    def send_hover_setpoint(self, hover_point):

        if self.context.ok():
            print(f"Sending hover setpoint {hover_point}")
            msg = Twist()
            msg.linear.x = hover_point[0]
            msg.linear.y = hover_point[1]
            msg.angular.z = hover_point[2]
            msg.linear.z = hover_point[3]
            self.publish_motion(msg)


class MotionProgram(Thread):
    def __init__(self, motion_commander: MotionCommander):
        Thread.__init__(self)
        self.motion_commander = motion_commander

    def run(self):
        time.sleep(1)
        self.motion_commander.take_off()
        time.sleep(5)
        self.motion_commander.forward(0.2)
        time.sleep(1)
        self.motion_commander.left(0.2)
        time.sleep(1)
        self.motion_commander.back(0.2)
        time.sleep(1)
        self.motion_commander.right(0.2)
        time.sleep(5)
        self.motion_commander.land()


def main(args=None):
    rclpy.init(args=args)

    motion_publisher = MotionPublisher()

    motion_commander = MotionCommander(motion_publisher)

    motion_program = MotionProgram(motion_commander)

    motion_program.start()

    rclpy.spin(motion_publisher)

    motion_program.join()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motion_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
