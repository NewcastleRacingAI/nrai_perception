import cv2
import numpy as np
from ultralytics import YOLO
import importlib.resources as resources

import math


class ZEDYOLOTrack():

    def __init__(self):
        self.model = YOLO(resources.files("nrai_perception.resource").joinpath("best.pt"))

        # ZED intrinsics (tune if needed)
        self.fx = 220.0
        self.fy = 700.0
        self.cx = 320.0
        self.cy = 320.0

        self.lens_depth = 210

        self.depth_frame: np.ndarray | None = None

    def depth_callback(self, frame: np.ndarray):
        self.depth_frame = frame

    def rgb_callback(self, frame: np.ndarray):
        if self.depth_frame is None:
            return
        
        frame = frame[:, :, :3]


        results = self.model.predict(frame, conf=0.4, verbose=False)

        cones = []

        # --- 1. Convert detections → 3D ---
        #print("new:")
        lap = [False, False]
        for det in results[0].boxes:
            x1,y1,x2,y2 = map(int, det.xyxy[0])
            cls = int(det.cls[0])

            cx = int((x1+x2)/2)
            cy = int((y1+y2)/2)

            depth = float(self.depth_frame[cy,cx])/150
            if np.isnan(depth) or depth < 0.3 or depth > 10:
                #print(f"triggered: {depth}")
                continue

            #lat_angle = math.atan((cx - self.cx)/self.lens_depth)

            #Z = depth * math.sin(lat_angle)
            #X = depth * math.cos(lat_angle)

            Z = depth
            X = (cx - self.cx) * Z / self.fx

            #print(f"({Z}, {X})")

            cones.append((cls, X, Z))
            # Draw
            match cls:
                case 0:
                    color = (255, 0, 0)      # Blue
                case 1:
                    color = (0, 0, 255)      # large_orange
                    
                    # Determine whether lap imminent
                    length = self.depth_frame.shape[0]
                    if cx < 0.4*length and not lap[0]:
                        lap[0] = True
                    if cx > 0.6*length and not lap[1]:
                        lap[1] = True
                case 2:
                    color = (0, 165, 255)    # Orange
                case 3:
                    color = (0, 255, 0)      # unknown
                case 4:
                    color = (0, 255, 255)    # Yellow
            #circled = cv2.circle(frame, (cx, cy), 4, color, -1)
            #cv2.imshow("Perception", circled)
            #cv2.waitKey(1)

            # cv2.circle(frame,(cx,cy),4,(0,255,0),-1)
        
        lap = lap[0] and lap[1]

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
        
        return [left, right], lap
