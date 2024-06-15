import os
import rclpy
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image as MsgImage
from PIL import Image as PILImage

cnt = 0  # number of images
timestamp = datetime.datetime.now()  # time when the previous img is saved


class Subscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber_img_saver")
        self.subscription = self.create_subscription(
            MsgImage, "image_raw", self.listener_callback, 10
        )
        self.get_logger().info("Start listening messages from 'image_raw'")

    def listener_callback(self, msg):
        global timestamp
        now = datetime.datetime.now()

        # Take a picture every 1 sec
        if (now - timestamp).seconds < 1:
            return
        timestamp = now

        img_arr = np.array(msg.data).reshape((msg.height, msg.width, 3))
        img = PILImage.fromarray(img_arr, "RGB")

        img_dir = "imgs"
        if not os.path.isdir(img_dir):
            os.makedirs(img_dir)

        now_str = now.strftime("%Y-%m-%d_%H:%M:%S.%f")
        img_path = os.path.join(img_dir, f"img_{now_str}.png")
        img.save(img_path)
        self.get_logger().info(f"saved image {img_path}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
