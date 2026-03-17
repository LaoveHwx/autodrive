class Config:
    SCREEN_WIDTH = 900
    SCREEN_HEIGHT = 600
    FPS = 60

    # 真实摄像头第一人称（行车记录仪风格）核心参数
    FOV = 820                    # 最接近真实前视镜头（不鱼眼、不隧道）
    VANISHING_POINT_Y = 260      # 地平线约画面 43% 高度，真实感最强

    # 道路参数（现实高速公路比例）
    LANE_WIDTH = 570
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
    
    LAMP_INTERVAL = 340          # 现实高速路灯间隔 ≈30–40 m
    LAMP_SIDE_OFFSET = 0.72