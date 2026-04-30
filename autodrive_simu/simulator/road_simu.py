import math
import random
import time
from collections import deque

import cv2
import numpy as np
import pygame

from autodrive_simu.config import Config
from autodrive_simu.perception.lane_detector import LaneDetector


class RoadPoint:
    def __init__(self, x: float, z: float, has_lamp: bool = False):
        self.x = x
        self.z = z
        self.has_lamp = has_lamp


class RoadModel:
    def __init__(self):
        self.points = deque()
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.traveled_z = 0.0
        self.next_lamp_z = Config.LAMP_INTERVAL
        self._init_points()

    def _init_points(self):
        for z in range(0, int(Config.FAR_CLIP + 800), 40):
            place_lamp = z >= self.next_lamp_z
            if place_lamp:
                self.next_lamp_z += Config.LAMP_INTERVAL
            self.points.append(RoadPoint(0.0, float(z), place_lamp))

    def update(self):
        if random.random() < Config.CURVE_CHANCE:
            self.target_heading = random.uniform(-Config.MAX_ANGLE, Config.MAX_ANGLE)

        self.current_heading += (self.target_heading - self.current_heading) * 0.018
        self.traveled_z += Config.ROAD_SPEED

        for point in self.points:
            point.z -= Config.ROAD_SPEED

        while self.points and self.points[0].z < -Config.NEAR_CLIP:
            self.points.popleft()

        while len(self.points) < 180:
            last = self.points[-1]
            new_z = last.z + 40
            new_x = last.x + math.sin(math.radians(self.current_heading)) * 40
            place_lamp = new_z >= self.next_lamp_z
            if place_lamp:
                self.next_lamp_z += Config.LAMP_INTERVAL
            self.points.append(RoadPoint(new_x, new_z, place_lamp))

    def get_center_x_at(self, query_z: float) -> float:
        if not self.points:
            return Config.SCREEN_WIDTH / 2
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

    def visible_segments(self):
        return [p for p in self.points if p.z > -Config.NEAR_CLIP]


class HoodCamera:
    def __init__(self):
        self.height = Config.CAMERA_HEIGHT
        self.forward_offset = Config.CAMERA_FORWARD_OFFSET
        self.pitch_rad = math.radians(Config.CAMERA_PITCH_DEG)
        self.smoothed_x = Config.SCREEN_WIDTH / 2

    def update(self, vehicle_x: float):
        self.smoothed_x += (vehicle_x - self.smoothed_x) * Config.CAMERA_X_SMOOTHNESS
        return self.smoothed_x

    def project(self, world_x: float, world_y: float, world_z: float, camera_x: float):
        rel_x = world_x - camera_x
        rel_y = world_y - self.height
        rel_z = world_z + self.forward_offset

        cos_p = math.cos(self.pitch_rad)
        sin_p = math.sin(self.pitch_rad)

        view_y = rel_y * cos_p - rel_z * sin_p
        view_z = rel_y * sin_p + rel_z * cos_p

        if view_z <= Config.NEAR_CLIP or view_z > Config.FAR_CLIP:
            return 0, 0, 0.0, False

        scale = Config.FOV / view_z
        screen_x = Config.SCREEN_WIDTH / 2 + rel_x * scale
        screen_y = Config.VANISHING_POINT_Y + view_y * scale
        return int(screen_x), int(screen_y), scale, True


class VehicleController:
    def __init__(self):
        self.x = Config.SCREEN_WIDTH / 2
        self.target_lane_id = 0
        self.prev_error = 0.0
        self.actual_steer = 0.0
        self.human_steer = 0.0
        self.override_fade = 0.0

    def update(self, keys, road: RoadModel):
        human_target = 0.0
        if keys[pygame.K_a]:
            human_target = -1.0
        elif keys[pygame.K_d]:
            human_target = 1.0

        self.human_steer += (human_target * Config.MAX_STEER_MANUAL - self.human_steer) * Config.HUMAN_RAMP_SPEED
        self.override_fade = min(1.0, self.override_fade + 0.12) if human_target != 0 else max(0.0, self.override_fade * 0.88)

        lookahead_z = Config.LOOKAHEAD_DIST
        road_center = road.get_center_x_at(lookahead_z)
        target_x = road_center + self.target_lane_id * Config.LANE_WIDTH

        error = target_x - self.x
        derivative = error - self.prev_error
        pid = Config.KP * error + Config.KD * derivative
        auto_steer = max(-Config.MAX_STEER_AUTO, min(Config.MAX_STEER_AUTO, pid))
        self.prev_error = error

        total_target = auto_steer * (1 - self.override_fade) + self.human_steer * self.override_fade
        self.actual_steer += (total_target - self.actual_steer) * Config.STEER_SMOOTHNESS
        self.x += self.actual_steer

        current_offset = self.x - road.get_center_x_at(0.1)
        current_lane_float = current_offset / Config.LANE_WIDTH
        nearest = max(-1, min(1, round(current_lane_float)))

        if abs(self.human_steer) > Config.LANE_CHANGE_THRESHOLD and nearest != self.target_lane_id:
            self.target_lane_id = nearest
        elif self.override_fade < 0.15 and abs(current_lane_float - self.target_lane_id) > 0.35:
            self.target_lane_id = nearest

    def draw(self, screen):
        hood_y = int(Config.SCREEN_HEIGHT * (1 - Config.HOOD_HEIGHT_RATIO))
        pygame.draw.rect(screen, Config.COLOR_HOOD, (0, hood_y, Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT - hood_y))
        pygame.draw.line(screen, Config.COLOR_HOOD_EDGE, (0, hood_y), (Config.SCREEN_WIDTH, hood_y), 3)


class RoadRenderer:
    def __init__(self):
        self.roi_poly = []

    def draw_background(self, screen):
        pygame.draw.rect(screen, Config.COLOR_SKY, (0, 0, Config.SCREEN_WIDTH, Config.VANISHING_POINT_Y))
        pygame.draw.rect(screen, Config.COLOR_GRASS, (0, Config.VANISHING_POINT_Y, Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT - Config.VANISHING_POINT_Y))
        pygame.draw.line(screen, (180, 220, 255), (0, Config.VANISHING_POINT_Y), (Config.SCREEN_WIDTH, Config.VANISHING_POINT_Y), 2)

    def build_projection_cache(self, road: RoadModel, camera: HoodCamera):
        cache = []
        camera_x = camera.smoothed_x
        for point in road.visible_segments():
            left = camera.project(point.x - Config.ROAD_WIDTH / 2, 0.0, point.z, camera_x)
            right = camera.project(point.x + Config.ROAD_WIDTH / 2, 0.0, point.z, camera_x)
            center = camera.project(point.x, 0.0, point.z, camera_x)
            if left[3] and right[3] and center[3]:
                cache.append({
                    "point": point,
                    "left": left,
                    "right": right,
                    "center": center,
                })
        return cache

    def draw_road(self, screen, cache):
        fog_color = np.array(Config.COLOR_FOG)
        road_color = np.array(Config.COLOR_ROAD)

        for i in range(len(cache) - 1):
            a = cache[i]
            b = cache[i + 1]
            poly = [a["left"][:2], b["left"][:2], b["right"][:2], a["right"][:2]]
            fog_factor = min(1.0, max(0, a["point"].z / Config.FAR_CLIP) ** 1.2)
            surface_color = tuple((road_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            pygame.draw.polygon(screen, surface_color, poly)

    def draw_lane_markings(self, screen, cache):
        line_color = np.array(Config.COLOR_LANE_LINE)
        fog_color = np.array(Config.COLOR_FOG)
        for i in range(len(cache) - 1):
            a = cache[i]
            b = cache[i + 1]
            if a["point"].z < 0 or b["point"].z < 0:
                continue
            fog_factor = min(1.0, max(0, a["point"].z / Config.FAR_CLIP) ** 1.2)
            mark_color = tuple((line_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            thickness = max(1, int(5 * (1 - fog_factor)))
            for j in range(1, 3):
                offset = (j - 1) * Config.LANE_WIDTH - Config.LANE_WIDTH
                ax = a["center"][0] + offset * a["center"][2]
                ay = a["center"][1]
                bx = b["center"][0] + offset * b["center"][2]
                by = b["center"][1]
                pygame.draw.line(screen, mark_color, (int(ax), ay), (int(bx), by), thickness)

    def draw_objects(self, screen, cache):
        fog_color = np.array(Config.COLOR_FOG)
        guard_color = np.array(Config.COLOR_GUARD)
        for data in reversed(cache):
            point = data["point"]
            if not point.has_lamp:
                continue
            scale = data["center"][2]
            if scale <= 0:
                continue
            pole_height = int(260 * scale)
            pole_width = max(1, int(6 * scale))
            left_x = int(data["left"][0] - Config.ROAD_WIDTH * 0.55 * scale)
            right_x = int(data["right"][0] + Config.ROAD_WIDTH * 0.55 * scale)
            y = data["left"][1]
            top_y = y - pole_height
            fog_factor = min(1.0, max(0, point.z / Config.FAR_CLIP))
            color = tuple((guard_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            pygame.draw.line(screen, color, (left_x, top_y), (left_x, y), pole_width)
            pygame.draw.line(screen, color, (right_x, top_y), (right_x, y), pole_width)

    def build_roi(self, road: RoadModel, camera: HoodCamera):
        near_z = Config.NEAR_CLIP * 1.5
        far_z = min(Config.FAR_CLIP * 0.3, 1800)
        center_near = road.get_center_x_at(near_z)
        center_far = road.get_center_x_at(far_z)
        left_near = camera.project(center_near - Config.ROAD_WIDTH * 0.9, 0.0, near_z, camera.smoothed_x)
        right_near = camera.project(center_near + Config.ROAD_WIDTH * 0.9, 0.0, near_z, camera.smoothed_x)
        left_far = camera.project(center_far - Config.ROAD_WIDTH * 0.6, 0.0, far_z, camera.smoothed_x)
        right_far = camera.project(center_far + Config.ROAD_WIDTH * 0.6, 0.0, far_z, camera.smoothed_x)
        if left_near[3] and right_near[3] and left_far[3] and right_far[3]:
            def clamp(x, y):
                return max(0, min(Config.SCREEN_WIDTH, x)), max(0, min(Config.SCREEN_HEIGHT, y))
            self.roi_poly = [
                clamp(left_near[0], left_near[1]),
                clamp(left_far[0], left_far[1]),
                clamp(right_far[0], right_far[1]),
                clamp(right_near[0], right_near[1])
            ]
        else:
            self.roi_poly = [(0, Config.SCREEN_HEIGHT), (Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT), (Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT // 2), (0, Config.SCREEN_HEIGHT // 2)]

    def draw(self, screen, road: RoadModel, camera: HoodCamera):
        self.draw_background(screen)
        self.build_roi(road, camera)
        cache = self.build_projection_cache(road, camera)
        self.draw_road(screen, cache)
        self.draw_objects(screen, cache)
        self.draw_lane_markings(screen, cache)


class PerceptionView:
    def __init__(self):
        self.last_mask = None

    def detect(self, frame, roi_poly):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 40, 120)

        mask = np.zeros_like(edges)
        pts = np.array([roi_poly], np.int32)
        cv2.fillPoly(mask, pts, 255)
        self.last_mask = mask

        masked = cv2.bitwise_and(edges, mask)
        resized = cv2.resize(masked, (Config.PERCEPTION_WIDTH, Config.PERCEPTION_HEIGHT), interpolation=cv2.INTER_LINEAR)
        return resized


class DebugHUD:
    def __init__(self):
        self.font = pygame.font.Font(None, 24)

    def draw(self, screen, vehicle: VehicleController, fps: float, render_ms: float, cv_ms: float):
        mode = "MANUAL" if vehicle.override_fade > 0.3 else "AD"
        color = Config.COLOR_HUD_ALERT if mode == "MANUAL" else Config.COLOR_HUD_OK
        texts = [
            f"MODE: {mode}",
            f"FPS: {int(fps)}",
            f"Render: {int(render_ms)}ms",
            f"CV: {int(cv_ms)}ms",
            f"Steer: {round(vehicle.actual_steer, 1)}°",
            f"Target Lane: {vehicle.target_lane_id}",
        ]
        for idx, text in enumerate(texts):
            screen.blit(self.font.render(text, True, color), (12, 12 + idx * 22))


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT))
        pygame.display.set_caption("AutoDrive - 引擎盖上方视角演示")
        self.clock = pygame.time.Clock()

        self.road = RoadModel()
        self.vehicle = VehicleController()
        self.camera = HoodCamera()
        self.renderer = RoadRenderer()
        self.perception = PerceptionView()
        self.hud = DebugHUD()
        self.detector = LaneDetector()

    def run(self):
        cv_ms = 0.0
        while True:
            frame_start = time.perf_counter()
            self.clock.tick(Config.FPS)
            keys = pygame.key.get_pressed()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            self.road.update()
            self.vehicle.update(keys, self.road)
            self.camera.update(self.vehicle.x)

            render_start = time.perf_counter()
            self.renderer.draw(self.screen, self.road, self.camera)
            self.vehicle.draw(self.screen)
            render_ms = (time.perf_counter() - render_start) * 1000.0

            fps = self.clock.get_fps()
            self.hud.draw(self.screen, self.vehicle, fps, render_ms, cv_ms)
            pygame.display.flip()

            cv_start = time.perf_counter()
            img = cv2.cvtColor(pygame.surfarray.array3d(self.screen).transpose([1, 0, 2]), cv2.COLOR_RGB2BGR)
            perception = self.perception.detect(img, self.renderer.roi_poly)
            cv2.imshow("CV Perception", perception)
            cv2.waitKey(1)
            cv_ms = (time.perf_counter() - cv_start) * 1000.0

            frame_ms = (time.perf_counter() - frame_start) * 1000.0
            if frame_ms > 16.7:
                pygame.display.set_caption(f"AutoDrive - {int(frame_ms)}ms/frame")
