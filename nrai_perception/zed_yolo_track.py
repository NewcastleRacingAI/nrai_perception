#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class ZEDYOLOTrack(Node):

    def __init__(self, image, depth, path):
        super().__init__('zed_yolo_track')

        self.bridge = CvBridge()
        self.model = YOLO(path)

        # ZED intrinsics (tune if needed)
        self.fx = 700.0
        self.fy = 700.0
        self.cx = 640.0
        self.cy = 360.0

        self.depth_frame = None

        self.create_subscription(Image, image, self.rgb_callback, 10)
        self.create_subscription(Image, depth, self.depth_callback, 10)
        
        self.publisher_cones_left = self.create_publisher(Path, '/nrai_perception/cones_left', 10)
        self.publisher_cones_right = self.create_publisher(Path, '/nrai_perception/cones_right', 10)

        self.get_logger().info("ZED YOLO Track Node Started")

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def rgb_callback(self, msg):
        if self.depth_frame is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(frame, conf=0.4, verbose=False)

        cones = []

        # --- 1. Convert detections → 3D ---
        for det in results[0].boxes:
            x1,y1,x2,y2 = map(int, det.xyxy[0])
            cls = int(det.cls[0])

            cx = int((x1+x2)/2)
            cy = int((y1+y2)/2)

            depth = float(self.depth_frame[cy,cx])
            if np.isnan(depth) or depth < 0.3 or depth > 20:
                continue

            Z = depth
            X = (cx - self.cx) * Z / self.fx

            cones.append((cls, X, Z))
            # Draw
            if cls == 0:
                color = (255, 0, 0)      # Blue
            elif cls == 1:
                color = (0, 0, 255)      # large_orange
            elif cls == 2:
                color = (0, 165, 255)    # Orange
            elif cls == 3:
                color = (0, 255, 0)      # unknown
            elif cls == 4:
                color = (0, 255, 255)    # Yellow
            cv2.circle(frame, (cx, cy), 4, color, -1)
           

            # cv2.circle(frame,(cx,cy),4,(0,255,0),-1)

        # --- 2. Separate left & right ---
        left = []
        right = []
        # label = ["Blue","large_orange","Orange","unknown","yellow"][c[0]]

        for c in cones:
            if c[0] == 0:   # blue
                left.append((c[1], c[2]))
            elif c[0] == 4: # yellow
                right.append((c[1], c[2]))

        left.sort(key=lambda p: p[1])               #sort based on Z value
        right.sort(key=lambda p: p[1])
        
        # --- 3. Broadcast cones ---
        
        self.publisher_cones_left.publish(self.cones_to_path(left))
        self.publisher_cones_right.publish(self.cones_to_path(right))
    
    def cones_to_path(self, cones):
        out_cones = Path()
        for c in cones:
            cone = Point()
            cone.x = float(c[0])
            cone.y = float(-0.22)
            cone.z = float(c[1])
            
            pose = Pose()
            pose.position = point

            poseStamped=PoseStamped()
            poseStamped.pose=pose
            
            out_cones.poses.append(poseStamped)
        
        return out_cones
        

def main(args=None):
    
    image = "/zed/zed_node/rgb/color/rect/image"
    depth = "/zed/zed_node/depth/depth_registered"
    path = "nrai_perception/resource/best.pt"
    
    rclpy.init(args=args)
    node = ZEDYOLOTrack(image, depth, path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

def eufs(args=None):
    
    # Make sure, when running in EUFS_SIM, to disable simulated perception.
    
    image = "/zed/image_raw"
    depth = "/zed/depth/image_raw"
    path = "../../../resource/best.pt"
    
    rclpy.init(args=args)
    node = ZEDYOLOTrack(image, depth, path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
