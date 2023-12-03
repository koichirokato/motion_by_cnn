#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import tensorflow as tf
from motion_by_cnn.utils import predict, make_cmdvel

IMAGE_MAX = 100


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("move_by_cnn")

        self.sub_img = self.create_subscription(
            Image, "/image", self.image_callback, rclpy.qos.qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.cv_bridge: CvBridge = CvBridge()
        self.labels = []
        with open("./labels.txt", "r") as f:
            for line in f:
                self.labels.append(line.rstrip())

        self.model_pred = tf.keras.models.load_model("./my_model.h5")
        self.current_label: str = ""
        self.next_motion: str = ""

    def image_callback(self, msg: Image) -> None:
        img_np = self.cv_bridge.imgmsg_to_cv2(msg)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        self.current_label = predict(img_np, self.model_pred, self.labels)
        cv2.putText(
            img_np,
            self.next_motion,
            (20, 50),
            cv2.FONT_HERSHEY_DUPLEX,
            1.0,
            (255, 255, 255),
        )
        cv2.imshow("Image", img_np)
        cv2.waitKey(1)

    def timer_callback(self):
        self.move(self.current_label)

    def move(self, label):
        twist = Twist()
        x, y, heading, self.next_motion = make_cmdvel(label)
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = heading

        self.cmd_pub.publish(twist)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImageSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
