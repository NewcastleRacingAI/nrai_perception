#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class ZEDYOLOTrack(Node):

    def __init__(self):
        super().__init__('zed_yolo_track')

        self.bridge = CvBridge()
        self.model = YOLO("nrai_perception/resource/best.pt")

        # ZED intrinsics (tune if needed)
        self.fx = 700.0
        self.fy = 700.0
        self.cx = 640.0
        self.cy = 360.0

        self.depth_frame = None

        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", self.rgb_callback, 10)
        self.create_subscription(Image, "/zed/zed_node/depth/depth_registered", self.depth_callback, 10)

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

        # --- 3. Compute midpoints ---
        midpoints = []

        for L in left:
            best = None
            min_d = 999
            for R in right:
                d = abs(L[1] - R[1])          #diff in z value
                if d < min_d:                
                    min_d = d
                    best = R

            if best:
                mx = (L[0] + best[0]) / 2
                mz = (L[1] + best[1]) / 2
                midpoints.append((mx, mz))
        print("Midpoints:", midpoints)
        # --- 4. Draw centerline ---
        for m in midpoints:
            X = m[0]
            Z = m[1]
            Y = -0.22  # assume ground level (20cm below camera)

            u = int(self.cx + X * self.fx / Z)
            v = int(self.cy + Y * self.fy / Z)

            print("Drawing at pixel:", u, v)
            cv2.circle(frame, (u,v), 5, (0,255,0), -1)


        cv2.imshow("Track", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ZEDYOLOTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
