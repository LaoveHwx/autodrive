import cv2
import numpy as np

def detect_lane(frame):
    if frame is None:
        return None, [], []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#灰度化
    blur = cv2.GaussianBlur(gray, (5, 5), 0)# 高斯模糊
    edges = cv2.Canny(blur, 50, 150)# 边缘检测

    height, width = edges.shape
    # 只画下半部分
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height * 0.6)),
        (0, int(height * 0.6))
    ]], dtype=np.int32)
    cv2.fillPoly(mask, [polygon], 255)

    # 提取感兴趣区域（ROI）
    roi = cv2.bitwise_and(edges, mask)
    # 霍夫直线检测
    lines = cv2.HoughLinesP(
        roi,
        1,
        np.pi/180,
        30,
        minLineLength=20,
        maxLineGap=50
    )
    
    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 防止除零错误
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            #  # 过滤斜率过小的直线（非车道线）
            # if abs(slope) < 0.5:
            #     continue
            # 区分左右车道线
            if slope < 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

    return roi, left_lines, right_lines