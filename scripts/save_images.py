#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import datetime

IMAGE_MAX = 100


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.sub_img: Node.subscriptions = self.create_subscription(
            Image, "/image", self.image_callback, rclpy.qos.qos_profile_sensor_data
        )

        self.timer: Node.timers = self.create_timer(2, self.timer_callback)

        self.cv_bridge: CvBridge = CvBridge()
        self.image_count: int = 0
        self.sub_image: Image = None

    def image_callback(self, msg: Image) -> None:
        self.sub_image = msg

    def timer_callback(self) -> None:
        self.image_count += 1
        if self.image_count <= IMAGE_MAX and self.sub_image is not None:
            img_np = self.cv_bridge.imgmsg_to_cv2(self.sub_image)
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)

            now = datetime.datetime.now()
            filename = "./images/log_" + now.strftime("%Y%m%d_%H%M%S") + ".jpg"
            self.get_logger().info(
                f"Save image : {filename}, [{self.image_count}/{IMAGE_MAX}]"
            )
            cv2.imwrite(filename, img_np)


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
