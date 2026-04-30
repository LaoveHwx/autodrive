class Config:
    SCREEN_WIDTH = 900
    SCREEN_HEIGHT = 600
    FPS = 60

    # 2.5D 引擎盖上方相机参数
    CAMERA_HEIGHT = 150.0
    CAMERA_FORWARD_OFFSET = 175.0
    CAMERA_PITCH_DEG = 18.0
    NEAR_CLIP = 60.0
    FAR_CLIP = 5000.0
    HOOD_HEIGHT_RATIO = 0.20
    CAMERA_X_SMOOTHNESS = 0.12

    # 真实摄像头第一人称（行车记录仪风格）核心参数
    FOV = 820                    # 最接近真实前视镜头（不鱼眼、不隧道）
    VANISHING_POINT_Y = 260      # 地平线约画面 43% 高度，真实感最强

    # 道路参数（现实高速公路比例）
    LANE_WIDTH = 240
    ROAD_WIDTH = LANE_WIDTH * 3
    ROAD_SPEED = 22              # 100–120 km/h 巡航最自然路面流动感
    CURVE_CHANCE = 0.008         # 现实高速弯道频率
    MAX_ANGLE = 30

    # 人机共驾 + 方向盘物理（最接近真实 LKA + 手动干预）
    KP = 0.11
    KD = 0.82
    LOOKAHEAD_DIST = 300

    MAX_STEER_MANUAL = 22.0
    MAX_STEER_AUTO = 14.0
    STEER_SMOOTHNESS = 0.18
    HUMAN_RAMP_SPEED = 0.14      # 持续按住转向渐进，更真实
    LANE_CHANGE_THRESHOLD = 10.0

    COLOR_SKY = (105, 175, 245)
    COLOR_GRASS = (38, 115, 38)
    COLOR_ROAD = (55, 55, 58)
    COLOR_ROAD_EDGE = (90, 90, 95)
    COLOR_LANE_LINE = (230, 230, 230)
    COLOR_GUARD = (100, 100, 110)
    COLOR_HOOD = (19, 19, 28)
    COLOR_HOOD_EDGE = (48, 48, 58)
    COLOR_HUD_TEXT = (240, 240, 240)
    COLOR_HUD_ALERT = (240, 80, 80)
    COLOR_HUD_OK = (80, 240, 100)
    COLOR_FOG = (38, 115, 38)

    LAMP_INTERVAL = 340          # 现实高速路灯间隔 ≈30–40 m
    LAMP_SIDE_OFFSET = 0.72
    GUARD_POST_SPACING = 220
    PERCEPTION_WIDTH = 450
    PERCEPTION_HEIGHT = 300
    ROI_MARGIN = 0.25
