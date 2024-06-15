import rclpy
import os
import numpy as np

import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_interface.srv import FindPose
from ultralytics import YOLO

# Load YOLO model
model = YOLO(os.path.expanduser("~/Proj/runs/detect/train21/weights/best.pt"))

# Camera intrinsic parameters (from the YAML file)
fx = 1552.7
fy = 1537.9
cx = 640.0
cy = 360.0
EDGE_SIZE = 0.2  # Known cube size in meters


class ObjDetector(Node):
    def __init__(self):
        super().__init__("obj_detector")
        self.ore_pos_x = 0.0
        self.ore_pos_y = 0.0
        self.ore_pos_z = 0.0
        self.last_rec = 0.0
        self.subscription = self.create_subscription(
            Image, "image_raw", self.listener_callback, 10
        )
        self.service = self.create_service(
            FindPose, "find_pos", self.service_callback
        )
        self.get_logger().info("ObjDetector initialized.")

    def listener_callback(self, msg):
        img_arr = np.array(msg.data).reshape((msg.height, msg.width, 3))
        result = model(img_arr)[0]

        # Return if no object is detected
        if result.boxes.xyxy.numel() <= 0:
            return
        
        last_rec = rclpy.clock.Clock().now().nanoseconds()

        self.get_logger().info("-" * 80)
        self.get_logger().info(f"result.boxes.xyxy: {result.boxes.xyxy}")

        for box in result.boxes.xyxy:
            x1, y1, x2, y2 = box
            pixel_width = x2 - x1
            pixel_height = y2 - y1

            # Assuming cube is facing camera squarely
            pixel_size = max(pixel_width, pixel_height)

            # Calculate real-world distance Z using similar triangles
            Z = (EDGE_SIZE * fx) / pixel_size

            # Calculate real-world coordinates X, Y (center of the bounding box)
            X = ((x1 + x2) / 2 - cx) * Z / fx
            Y = ((y1 + y2) / 2 - cy) * Z / fy

            self.get_logger().info(f"X: {X:.2f}m, Y: {Y:.2f}m, Z: {Z:.2f}m")

            self.ore_pos_x = X
            self.ore_pos_y = Y
            self.ore_pos_z = Z
    
    def service_callback(self, request):
        response = FindPose.Response()
        if request.name == "ore":
            remain_valid = 0.5 # seconds
            if rclpy.clock.Clock().now().nanoseconds() - self.last_rec < remain_valid * 1e9:
                response.valid = True
                response.x = self.ore_pos_x
                response.y = self.ore_pos_y
                response.z = self.ore_pos_z
            else: # expired
                response.valid = False
        else: # other objects
            response.valid = False
        return response


def main(args=None):
    rclpy.init(args=args)
    od = ObjDetector()
    rclpy.spin(od)

    # Destroy the node explicitly
    od.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
