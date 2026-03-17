import cv2
import numpy as np

class LaneDetector:
    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 40, 120)

        h, w = edges.shape
        mask = np.zeros_like(edges)
        # 梯形 ROI 适配真实第一视角
        pts = np.array([[(0, h), (w, h), (w//2 + 95, 295), (w//2 - 95, 295)]], np.int32)
        cv2.fillPoly(mask, pts, 255) # type: ignore
        return cv2.bitwise_and(edges, mask)