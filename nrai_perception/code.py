import cv2
import numpy as np
from ultralytics import YOLO

class ZEDYOLOTrack():

    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO('../resource/best.pt')

        # ZED intrinsics (tune if needed)
        self.fx = 700.0
        self.fy = 700.0
        self.cx = 640.0
        self.cy = 360.0

        self.depth_frame = None

    def depth_callback(self, frame):
        self.depth_frame = frame

    def rgb_callback(self, frame):
        if self.depth_frame is None:
            return
        
        frame = frame[:, :, :3]

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
        
        # --- 3. Return cones ---
        
        return [left, right]
    
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