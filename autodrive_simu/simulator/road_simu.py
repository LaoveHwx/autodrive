import numpy as np
import pygame
import math
import random
import cv2
from collections import deque
from autodrive_simu.config import Config
from autodrive_simu.perception.lane_detector import LaneDetector


class RoadPoint:
    def __init__(self, x: float, z: float, has_lamp: bool = False):
        self.x = x
        self.z = z
        self.has_lamp = has_lamp


class Road:
    def __init__(self):
        self.points = deque()
        self.current_heading = 0.0
        self.target_heading = 0.0

        self.lamp_interval_z = 350  # 每隔多少深度生成一根路灯
        self.next_lamp_z = 350
        # 用于精准计算虚线滚动的真实物理距离
        self.traveled_z = 0.0
        # 初始化深度路点（真实 3D）
        for z in range(0, 6200, 40):
            place_lamp = False
            if z >= self.next_lamp_z:
                place_lamp = True
                self.next_lamp_z += self.lamp_interval_z
            self.points.append(RoadPoint(400.0, float(z), place_lamp))

    def update(self):
        if random.random() < Config.CURVE_CHANCE:
            self.target_heading = random.uniform(-Config.MAX_ANGLE, Config.MAX_ANGLE)

        # 更平滑弯道（减少变形）
        self.current_heading += (self.target_heading - self.current_heading) * 0.018

        for p in self.points:
            p.z -= Config.ROAD_SPEED

        while self.points and self.points[0].z < -100:
            self.points.popleft()

        # 生成新路点时，同时生成附着在它上面的路灯
        while len(self.points) < 160:
            last = self.points[-1]
            new_z = last.z + 40
            new_x = last.x + math.sin(math.radians(self.current_heading)) * 40

            place_lamp = False
            if new_z >= self.next_lamp_z:
                place_lamp = True
                self.next_lamp_z += self.lamp_interval_z

            self.points.append(RoadPoint(new_x, new_z, place_lamp))

    def get_center_x_at(self, query_z: float) -> float:
        """用于 AD 寻迹中心线"""
        if not self.points:
            return 400.0
        pts = list(self.points)
        if query_z <= pts[0].z:
            return pts[0].x
        if query_z >= pts[-1].z:
            return pts[-1].x

        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            if p1.z <= query_z <= p2.z:
                t = (query_z - p1.z) / (p2.z - p1.z)
                return p1.x + t * (p2.x - p1.x)
        return pts[-1].x

    def project(self, world_x: float, world_z: float, camera_x: float):
        safe_z = max(0.1, world_z)
        scale = Config.FOV / (safe_z + Config.FOV)          # 平直真实镜头

        rel_x = world_x - camera_x
        screen_x = Config.SCREEN_WIDTH // 2 + rel_x * scale

        # 真实线性透视（近大远小，平直感强）
        screen_y = Config.VANISHING_POINT_Y + (Config.SCREEN_HEIGHT - Config.VANISHING_POINT_Y) * scale

        return int(screen_x), int(screen_y)

    def draw(self, screen, camera_x: float):
        # 天空与草地
        pygame.draw.rect(screen, (135, 206, 235), (0, 0, 800, Config.VANISHING_POINT_Y))
        pygame.draw.rect(screen, (34, 100, 34), (0, Config.VANISHING_POINT_Y, 800, 600))

        if len(self.points) < 2:
            return

        # 基础材质颜色 (使用 numpy 方便颜色插值运算)
        fog_color = np.array([34, 100, 34])       # 远处的雾色（与草地一致）
        road_base_color = np.array([55, 55, 58])  # 柏油路色
        line_base_color = np.array([230, 230, 230])

        max_z = 5000.0  # 雾化极限距离

        # --- 核心修复 3：分段梯形渲染 + 景深雾化 ---
        pts = list(self.points)
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i+1]

            # 如果整段都在镜头后，跳过
            if p2.z < 0:
                continue

            # 计算雾化系数 (Z 越大，越趋近于 1。1 代表完全融入背景)
            fog_factor = min(1.0, (max(0, p1.z) / max_z) ** 1.3)

            # 道路梯形顶点计算
            lx1, ly1 = self.project(p1.x - Config.ROAD_WIDTH // 2, p1.z, camera_x)
            rx1, ry1 = self.project(p1.x + Config.ROAD_WIDTH // 2, p1.z, camera_x)
            lx2, ly2 = self.project(p2.x - Config.ROAD_WIDTH // 2, p2.z, camera_x)
            rx2, ry2 = self.project(p2.x + Config.ROAD_WIDTH // 2, p2.z, camera_x)

            # 渲染路面分段 - 修复：颜色转换为整数元组
            r_color = road_base_color * (1 - fog_factor) + fog_color * fog_factor
            pygame.draw.polygon(screen, tuple(r_color.astype(int)), [(lx1, ly1), (lx2, ly2), (rx2, ry2), (rx1, ry1)])

            # 渲染真实物理虚线 (基于物理坐标 p1.z 与 行驶总里程)
            if int((p1.z + self.traveled_z) / 120) % 2 == 0:
                l_color = line_base_color * (1 - fog_factor) + fog_color * fog_factor
                line_thickness = max(1, int(4 * (1 - fog_factor)))

                for j in range(1, 3):
                    offset = -Config.ROAD_WIDTH // 2 + j * Config.LANE_WIDTH
                    cx1, cy1 = self.project(p1.x + offset, p1.z, camera_x)
                    cx2, cy2 = self.project(p2.x + offset, p2.z, camera_x)
                    # 修复：颜色转换为整数元组
                    pygame.draw.line(screen, tuple(l_color.astype(int)), (cx1, cy1), (cx2, cy2), line_thickness)

        # 绘制路灯
        lamp_width = 13
        lamp_height = 350

        for p in reversed(pts):
            if not p.has_lamp or p.z < 0:
                continue

            fog_factor = min(1.0, (p.z / max_z) ** 1.2)
            # 修复：颜色转换为整数元组
            pole_color = np.array([140, 40, 40]) * (1 - fog_factor) + fog_color * fog_factor
            bulb_color = np.array([255, 255, 120]) * (1 - fog_factor) + fog_color * fog_factor

            scale = Config.FOV / (p.z + Config.FOV)
            draw_thickness = max(1, int(lamp_width * scale))

            # 左侧路灯
            left_lx = p.x - Config.ROAD_WIDTH * 0.65
            llx, lly = self.project(left_lx, p.z, camera_x)
            lly_top = lly - lamp_height * scale
            if lly_top > Config.VANISHING_POINT_Y - 50:
                pygame.draw.line(screen, tuple(pole_color.astype(int)), (llx, lly_top), (llx, lly), draw_thickness)
                pygame.draw.circle(screen, tuple(bulb_color.astype(int)), (int(llx), int(lly_top)), max(2, draw_thickness + 2))

            # 右侧路灯
            right_lx = p.x + Config.ROAD_WIDTH * 0.65
            rlx, rly = self.project(right_lx, p.z, camera_x)
            rly_top = rly - lamp_height * scale
            if rly_top > Config.VANISHING_POINT_Y - 50:
                pygame.draw.line(screen, tuple(pole_color.astype(int)), (rlx, rly_top), (rlx, rly), draw_thickness)
                pygame.draw.circle(screen, tuple(bulb_color.astype(int)), (int(rlx), int(rly_top)), max(2, draw_thickness + 2))


class Vehicle:
    def __init__(self):
        self.x = 400.0
        self.target_lane_id = 0                     # -1 左 / 0 中 / 1 右
        self.prev_error = 0.0
        self.actual_steer = 0.0
        self.human_steer = 0.0                      # 持续输入
        self.override_fade = 0.0                    # 软接管过渡

    def update(self, keys, road):
        # 持续按住 A/D（多档位渐进转向）
        human_target = 0.0
        if keys[pygame.K_a]:
            human_target = -1.0
        elif keys[pygame.K_d]:
            human_target = 1.0

        # 渐进加速 + 指数回正
        self.human_steer += (human_target * Config.MAX_STEER_MANUAL - self.human_steer) * Config.HUMAN_RAMP_SPEED
        self.override_fade = min(1.0, self.override_fade + 0.12) if human_target != 0 else max(0.0, self.override_fade * 0.88)

        # 自动驾驶 PID（始终居中）
        lookahead_z = Config.LOOKAHEAD_DIST
        road_center = road.get_center_x_at(lookahead_z)
        target_x = road_center + self.target_lane_id * Config.LANE_WIDTH

        error = target_x - self.x
        derivative = error - self.prev_error
        pid = Config.KP * error + Config.KD * derivative
        auto_steer = max(-Config.MAX_STEER_AUTO, min(Config.MAX_STEER_AUTO, pid))
        self.prev_error = error

        # 总转向 = 自动 + 人为（软混合）
        total_target = auto_steer * (1 - self.override_fade) + self.human_steer * self.override_fade
        self.actual_steer += (total_target - self.actual_steer) * Config.STEER_SMOOTHNESS

        self.x += self.actual_steer

        # 软吸附 + 智能换道（松手后自动回正）
        current_offset = self.x - road.get_center_x_at(0.1)
        current_lane_float = current_offset / Config.LANE_WIDTH
        nearest = max(-1, min(1, round(current_lane_float)))

        if abs(self.human_steer) > Config.LANE_CHANGE_THRESHOLD and nearest != self.target_lane_id:
            self.target_lane_id = nearest
        elif self.override_fade < 0.15 and abs(current_lane_float - self.target_lane_id) > 0.35:
            # 软吸附：轻微修正目标车道
            self.target_lane_id = nearest

    def draw(self, screen):
        hood_offset = -self.actual_steer * 2.1
        hood_pts = [(30 + hood_offset, 600), (875 + hood_offset, 600),
                    (720 + hood_offset, 490), (180 + hood_offset, 490)]
        pygame.draw.polygon(screen, (19, 19, 28), hood_pts)
        pygame.draw.polygon(screen, (48, 48, 58), hood_pts, 3)

        wheel_center = (450 + hood_offset * 0.4, 555)
        pygame.draw.circle(screen, (14, 14, 14), wheel_center, 86, 18)

        rot = math.radians(self.actual_steer * 8.2)
        sx = math.cos(rot) * 78
        sy = math.sin(rot) * 78
        pygame.draw.line(screen, (80, 80, 80),
                         (wheel_center[0] - sx, wheel_center[1] - sy),
                         (wheel_center[0] + sx, wheel_center[1] + sy), 16)


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT))
        pygame.display.set_caption("AutoDrive - 真实第一人称（平直镜头）")
        self.clock = pygame.time.Clock()

        self.road = Road()
        self.vehicle = Vehicle()
        self.detector = LaneDetector()

        self.font = pygame.font.Font(None, 28)

    def run(self):
        while True:
            self.clock.tick(Config.FPS)
            keys = pygame.key.get_pressed()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            self.road.update()
            self.vehicle.update(keys, self.road)

            self.road.draw(self.screen, self.vehicle.x)
            self.vehicle.draw(self.screen)

            # HUD
            mode = "MANUAL" if self.vehicle.override_fade > 0.3 else "AD"
            color = (255, 60, 60) if mode == "MANUAL" else (80, 255, 80)
            self.screen.blit(self.font.render(f"MODE: {mode}  |  Steer: {round(self.vehicle.actual_steer,1)}°  |  Lane: {self.vehicle.target_lane_id}", True, color), (15, 18))

            pygame.display.flip()

            # CV 感知 
            img = cv2.cvtColor(pygame.surfarray.array3d(self.screen).transpose([1,0,2]), cv2.COLOR_RGB2BGR)
            cv2.imshow("CV Perception", self.detector.detect(img))
            cv2.waitKey(1)