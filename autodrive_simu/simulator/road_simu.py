import math
import time
from collections import deque

import cv2
import numpy as np
import pygame

from autodrive_simu.config import Config
from autodrive_simu.perception.lane_detector import LaneDetector


class RoadPoint:
    def __init__(
        self,
        x: float,
        z: float,
        s: float,
        heading: float = 0.0,
        has_lamp: bool = False,
        has_guard: bool = False,
        has_sign: bool = False,
    ):
        self.x = x
        self.z = z
        self.s = s
        self.heading = heading
        self.has_lamp = has_lamp
        self.has_guard = has_guard
        self.has_sign = has_sign


class RoadModel:
    def __init__(self):
        self.points = deque()
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.traveled_z = 0.0
        self.next_lamp_s = Config.LAMP_INTERVAL
        self.next_guard_s = Config.GUARD_POST_SPACING
        self.next_sign_s = Config.SIGN_INTERVAL
        self._init_points()

    def _center_x_for_s(self, s: float) -> float:
        loop_t = (s % Config.TRACK_LOOP_LENGTH) / Config.TRACK_LOOP_LENGTH
        angle = math.tau * loop_t
        return (
            math.sin(angle) * Config.TRACK_CURVE_AMPLITUDE
            + (math.sin(angle * 2.0 + 0.7) - math.sin(0.7)) * Config.TRACK_CURVE_SECONDARY_AMPLITUDE
        )

    def _heading_for_s(self, s: float) -> float:
        sample = 80.0
        dx = self._center_x_for_s(s + sample) - self._center_x_for_s(s - sample)
        return math.degrees(math.atan2(dx, sample * 2.0))

    def _make_point(self, relative_z: float, s: float) -> RoadPoint:
        has_lamp = s >= self.next_lamp_s
        if has_lamp:
            self.next_lamp_s += Config.LAMP_INTERVAL

        has_guard = s >= self.next_guard_s
        if has_guard:
            self.next_guard_s += Config.GUARD_POST_SPACING

        has_sign = s >= self.next_sign_s
        if has_sign:
            self.next_sign_s += Config.SIGN_INTERVAL

        return RoadPoint(
            self._center_x_for_s(s),
            relative_z,
            s,
            self._heading_for_s(s),
            has_lamp,
            has_guard,
            has_sign,
        )

    def _init_points(self):
        for z in range(0, int(Config.FAR_CLIP + 800), Config.ROAD_SAMPLE_STEP):
            self.points.append(self._make_point(float(z), float(z)))

    def update(self):
        self.traveled_z += Config.ROAD_SPEED
        self.target_heading = self.get_heading_at(Config.LOOKAHEAD_DIST)
        self.current_heading += (self.target_heading - self.current_heading) * 0.08

        for point in self.points:
            point.z -= Config.ROAD_SPEED

        while self.points and self.points[0].z < -Config.NEAR_CLIP:
            self.points.popleft()

        while len(self.points) < 180:
            last = self.points[-1]
            new_z = last.z + Config.ROAD_SAMPLE_STEP
            new_s = last.s + Config.ROAD_SAMPLE_STEP
            self.points.append(self._make_point(new_z, new_s))

    def get_center_x_at(self, query_z: float) -> float:
        if not self.points:
            return Config.WORLD_CENTER_X
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

    def get_heading_at(self, query_z: float) -> float:
        ahead = self.get_center_x_at(query_z + 80.0)
        behind = self.get_center_x_at(max(0.0, query_z - 80.0))
        return math.degrees(math.atan2(ahead - behind, 160.0))


class HoodCamera:
    def __init__(self):
        self.height = Config.CAMERA_HEIGHT
        self.forward_offset = Config.CAMERA_FORWARD_OFFSET
        self.pitch_rad = math.radians(Config.CAMERA_PITCH_DEG)
        self.smoothed_x = Config.WORLD_CENTER_X

    def update(self, vehicle_x: float):
        self.smoothed_x += (vehicle_x - self.smoothed_x) * Config.CAMERA_X_SMOOTHNESS
        return self.smoothed_x

    def project(self, world_x: float, world_y: float, world_z: float, camera_x: float):
        rel_x = world_x - camera_x
        rel_y = self.height - world_y
        rel_z = world_z + self.forward_offset

        cos_p = math.cos(self.pitch_rad)
        sin_p = math.sin(self.pitch_rad)

        view_y = rel_y * cos_p
        view_z = rel_z * cos_p + rel_y * sin_p

        if view_z <= Config.NEAR_CLIP or view_z > Config.FAR_CLIP:
            return 0, 0, 0.0, False

        scale = Config.FOV / view_z
        screen_x = Config.SCREEN_WIDTH / 2 + rel_x * scale
        screen_y = Config.VANISHING_POINT_Y + view_y * scale
        return int(screen_x), int(screen_y), scale, True


class VehicleController:
    def __init__(self):
        self.x = Config.WORLD_CENTER_X
        self.target_lane_id = 0
        self.prev_error = 0.0
        self.actual_steer = 0.0
        self.human_steer = 0.0
        self.override_fade = 0.0

    @staticmethod
    def _key_down(keys, *key_codes):
        return any(keys[key_code] for key_code in key_codes)

    def update(self, keys, road: RoadModel):
        human_target = 0.0
        if self._key_down(keys, pygame.K_a, pygame.K_LEFT):
            human_target = -1.0
        elif self._key_down(keys, pygame.K_d, pygame.K_RIGHT):
            human_target = 1.0

        self.human_steer += (human_target * Config.MAX_STEER_MANUAL - self.human_steer) * Config.HUMAN_RAMP_SPEED
        self.override_fade = (
            min(1.0, self.override_fade + Config.OVERRIDE_ATTACK)
            if human_target != 0
            else max(0.0, self.override_fade * Config.OVERRIDE_RELEASE)
        )

        lookahead_z = Config.LOOKAHEAD_DIST
        road_center = road.get_center_x_at(lookahead_z)
        target_x = road_center + self.target_lane_id * Config.LANE_WIDTH

        error = target_x - self.x
        derivative = error - self.prev_error
        curve_feed_forward = road.get_heading_at(lookahead_z) * Config.CURVE_FEED_FORWARD
        recovery_boost = 1.0 + (0.35 if self.override_fade < 0.05 and abs(error) > Config.LANE_WIDTH * 0.15 else 0.0)
        pid = (Config.KP * recovery_boost) * error + Config.KD * derivative + curve_feed_forward
        auto_steer = max(-Config.MAX_STEER_AUTO, min(Config.MAX_STEER_AUTO, pid))
        self.prev_error = error

        total_target = auto_steer * (1 - self.override_fade) + self.human_steer * self.override_fade
        self.actual_steer += (total_target - self.actual_steer) * Config.STEER_SMOOTHNESS
        self.x += self.actual_steer

        current_offset = self.x - road.get_center_x_at(0.1)
        road_limit = Config.ROAD_WIDTH * 0.5 - Config.LANE_WIDTH * 0.38
        self.x = road.get_center_x_at(0.1) + max(-road_limit, min(road_limit, current_offset))

    def draw(self, screen):
        hood_y = int(Config.SCREEN_HEIGHT * (1 - Config.HOOD_HEIGHT_RATIO))
        sway = int(self.actual_steer * 0.6)
        hood_poly = [
            (0, Config.SCREEN_HEIGHT),
            (0, hood_y + 26),
            (Config.SCREEN_WIDTH // 2 + sway, hood_y),
            (Config.SCREEN_WIDTH, hood_y + 26),
            (Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT),
        ]
        pygame.draw.polygon(screen, Config.COLOR_HOOD, hood_poly)
        pygame.draw.lines(screen, Config.COLOR_HOOD_EDGE, False, hood_poly[1:4], 3)
        ridge_x = Config.SCREEN_WIDTH // 2 + sway
        pygame.draw.line(screen, (58, 58, 72), (ridge_x, hood_y + 8), (ridge_x, Config.SCREEN_HEIGHT), 2)
        pygame.draw.arc(
            screen,
            (38, 38, 50),
            (int(Config.SCREEN_WIDTH * 0.18) + sway, hood_y + 18, int(Config.SCREEN_WIDTH * 0.64), 120),
            math.radians(190),
            math.radians(350),
            3,
        )


class RoadRenderer:
    def __init__(self):
        self.roi_poly = []

    def draw_background(self, screen):
        for y in range(0, Config.VANISHING_POINT_Y, 3):
            t = y / max(1, Config.VANISHING_POINT_Y)
            color = (
                int(92 + 48 * t),
                int(165 + 38 * t),
                int(235 + 15 * t),
            )
            pygame.draw.rect(screen, color, (0, y, Config.SCREEN_WIDTH, 3))
        pygame.draw.rect(screen, Config.COLOR_GRASS, (0, Config.VANISHING_POINT_Y, Config.SCREEN_WIDTH, Config.SCREEN_HEIGHT - Config.VANISHING_POINT_Y))
        pygame.draw.polygon(
            screen,
            (55, 132, 82),
            [(0, Config.VANISHING_POINT_Y + 34), (180, Config.VANISHING_POINT_Y + 5), (350, Config.VANISHING_POINT_Y + 28),
             (520, Config.VANISHING_POINT_Y + 2), (720, Config.VANISHING_POINT_Y + 30), (Config.SCREEN_WIDTH, Config.VANISHING_POINT_Y + 8),
             (Config.SCREEN_WIDTH, Config.VANISHING_POINT_Y + 78), (0, Config.VANISHING_POINT_Y + 78)],
        )
        pygame.draw.line(screen, (180, 220, 255), (0, Config.VANISHING_POINT_Y), (Config.SCREEN_WIDTH, Config.VANISHING_POINT_Y), 2)

    def build_projection_cache(self, road: RoadModel, camera: HoodCamera):
        cache = []
        camera_x = camera.smoothed_x
        for point in road.visible_segments():
            left = camera.project(point.x - Config.ROAD_WIDTH / 2, 0.0, point.z, camera_x)
            right = camera.project(point.x + Config.ROAD_WIDTH / 2, 0.0, point.z, camera_x)
            center = camera.project(point.x, 0.0, point.z, camera_x)
            guard_left = camera.project(point.x - Config.ROAD_WIDTH * Config.OBJECT_SIDE_OFFSET, 0.0, point.z, camera_x)
            guard_right = camera.project(point.x + Config.ROAD_WIDTH * Config.OBJECT_SIDE_OFFSET, 0.0, point.z, camera_x)
            if left[3] and right[3] and center[3]:
                cache.append({
                    "point": point,
                    "left": left,
                    "right": right,
                    "center": center,
                    "guard_left": guard_left,
                    "guard_right": guard_right,
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
            band = int(a["point"].s / Config.ROAD_TEXTURE_SPACING) % 2
            texture = 8 if band == 0 else -5
            textured_road = np.clip(road_color + texture, 0, 255)
            surface_color = tuple((textured_road * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            pygame.draw.polygon(screen, surface_color, poly)

    def draw_road_edges(self, screen, cache):
        edge_color = np.array(Config.COLOR_ROAD_EDGE)
        fog_color = np.array(Config.COLOR_FOG)

        for i in range(len(cache) - 1):
            a = cache[i]
            b = cache[i + 1]
            if a["point"].z < 0 or b["point"].z < 0:
                continue
            fog_factor = min(1.0, max(0, a["point"].z / Config.FAR_CLIP) ** 1.15)
            color = tuple((edge_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            thickness = max(1, int(4 * (1 - fog_factor)))
            pygame.draw.line(screen, color, a["left"][:2], b["left"][:2], thickness)
            pygame.draw.line(screen, color, a["right"][:2], b["right"][:2], thickness)

            stripe_phase = int(a["point"].s / 55) % 2
            if stripe_phase == 0 and a["point"].z < 1300:
                stripe_color = tuple((np.array((230, 215, 95)) * (1 - fog_factor) + fog_color * fog_factor).astype(int))
                for side in ("left", "right"):
                    center_x = a[side][0]
                    center_y = a[side][1]
                    offset = -1 if side == "left" else 1
                    pygame.draw.line(
                        screen,
                        stripe_color,
                        (int(center_x), int(center_y)),
                        (int(center_x + offset * 16 * a["center"][2]), int(center_y + 7)),
                        max(1, thickness),
                    )

    def draw_lane_markings(self, screen, cache):
        line_color = np.array(Config.COLOR_LANE_LINE)
        fog_color = np.array(Config.COLOR_FOG)
        for i in range(len(cache) - 1):
            a = cache[i]
            b = cache[i + 1]
            if a["point"].z < 0 or b["point"].z < 0:
                continue
            dash_period = Config.LANE_DASH_LENGTH + Config.LANE_DASH_GAP
            if ((a["point"].s + b["point"].s) * 0.5) % dash_period > Config.LANE_DASH_LENGTH:
                continue
            fog_factor = min(1.0, max(0, a["point"].z / Config.FAR_CLIP) ** 1.2)
            mark_color = tuple((line_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            thickness = max(1, int(6 * (1 - fog_factor)))
            for offset in (-Config.LANE_WIDTH / 2, Config.LANE_WIDTH / 2):
                ax = a["center"][0] + offset * a["center"][2]
                ay = a["center"][1]
                bx = b["center"][0] + offset * b["center"][2]
                by = b["center"][1]
                pygame.draw.line(screen, mark_color, (int(ax), ay), (int(bx), by), thickness)

    def draw_objects(self, screen, cache):
        fog_color = np.array(Config.COLOR_FOG)
        guard_color = np.array(Config.COLOR_GUARD)
        sign_green = np.array((35, 125, 95))
        reflector = np.array((245, 230, 120))

        for data in reversed(cache):
            point = data["point"]
            scale = data["center"][2]
            if scale <= 0:
                continue

            fog_factor = min(1.0, max(0, point.z / Config.FAR_CLIP))
            color = tuple((guard_color * (1 - fog_factor) + fog_color * fog_factor).astype(int))
            glow_color = tuple((reflector * (1 - fog_factor) + fog_color * fog_factor).astype(int))

            if point.has_guard and point.z < 2200:
                for side in ("guard_left", "guard_right"):
                    base = data[side]
                    if not base[3]:
                        continue
                    x, y = base[:2]
                    post_h = max(5, int(78 * scale))
                    post_w = max(2, int(5 * scale))
                    pygame.draw.rect(screen, color, (int(x - post_w / 2), int(y - post_h), post_w, post_h))
                    pygame.draw.rect(screen, glow_color, (int(x - post_w / 2), int(y - post_h * 0.72), post_w, max(2, post_w)))

                if abs(point.heading) > 4.5 and point.z < 1600:
                    side = "guard_right" if point.heading > 0 else "guard_left"
                    base = data[side]
                    if base[3]:
                        x, y = base[:2]
                        panel_w = max(9, int(34 * scale))
                        panel_h = max(8, int(28 * scale))
                        panel_color = tuple((np.array((235, 205, 55)) * (1 - fog_factor) + fog_color * fog_factor).astype(int))
                        pygame.draw.rect(screen, panel_color, (int(x - panel_w / 2), int(y - panel_h * 2.5), panel_w, panel_h), border_radius=2)
                        arrow_dir = 1 if point.heading > 0 else -1
                        arrow = [
                            (int(x - arrow_dir * panel_w * 0.28), int(y - panel_h * 1.85)),
                            (int(x + arrow_dir * panel_w * 0.18), int(y - panel_h * 2.2)),
                            (int(x + arrow_dir * panel_w * 0.18), int(y - panel_h * 1.5)),
                        ]
                        pygame.draw.polygon(screen, (35, 35, 38), arrow)

            if point.has_lamp:
                pole_height = int(270 * scale)
                pole_width = max(1, int(6 * scale))
                left_x = int(data["guard_left"][0])
                right_x = int(data["guard_right"][0])
                y = data["guard_left"][1]
                top_y = y - pole_height
                pygame.draw.line(screen, color, (left_x, top_y), (left_x, y), pole_width)
                pygame.draw.line(screen, color, (right_x, top_y), (right_x, y), pole_width)
                arm = max(10, int(56 * scale))
                pygame.draw.line(screen, color, (left_x, top_y), (left_x + arm, top_y + int(10 * scale)), pole_width)
                pygame.draw.line(screen, color, (right_x, top_y), (right_x - arm, top_y + int(10 * scale)), pole_width)
                pygame.draw.circle(screen, glow_color, (left_x + arm, top_y + int(12 * scale)), max(2, int(8 * scale)))
                pygame.draw.circle(screen, glow_color, (right_x - arm, top_y + int(12 * scale)), max(2, int(8 * scale)))

            if point.has_sign and point.z < 2600:
                base = data["guard_right"] if int(point.s / Config.SIGN_INTERVAL) % 2 == 0 else data["guard_left"]
                if base[3]:
                    x, y = base[:2]
                    sign_w = max(18, int(118 * scale))
                    sign_h = max(10, int(52 * scale))
                    pole_h = max(18, int(90 * scale))
                    sign_color = tuple((sign_green * (1 - fog_factor) + fog_color * fog_factor).astype(int))
                    pygame.draw.line(screen, color, (x, y), (x, y - pole_h), max(1, int(4 * scale)))
                    pygame.draw.rect(screen, sign_color, (int(x - sign_w / 2), int(y - pole_h - sign_h), sign_w, sign_h), border_radius=2)
                    pygame.draw.line(screen, (225, 245, 230), (int(x - sign_w * 0.35), int(y - pole_h - sign_h * 0.45)), (int(x + sign_w * 0.35), int(y - pole_h - sign_h * 0.45)), max(1, int(3 * scale)))

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
        self.draw_road_edges(screen, cache)
        self.draw_lane_markings(screen, cache)
        self.draw_objects(screen, cache)


class PerceptionView:
    def __init__(self):
        self.last_mask = None

    def detect(self, frame, roi_poly):
        if roi_poly is None or len(roi_poly) < 4:
            h, w = frame.shape[:2]
            roi_poly = [(0, h), (w, h), (w, h // 2), (0, h // 2)]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 40, 120)

        mask = np.zeros_like(edges)
        pts = np.array([roi_poly], np.int32)
        cv2.fillPoly(mask, pts, 255)
        self.last_mask = mask

        masked = cv2.bitwise_and(edges, mask)
        preview = cv2.resize(frame, (Config.PERCEPTION_WIDTH, Config.PERCEPTION_HEIGHT), interpolation=cv2.INTER_LINEAR)
        preview = cv2.convertScaleAbs(preview, alpha=0.45, beta=0)
        edge_preview = cv2.resize(masked, (Config.PERCEPTION_WIDTH, Config.PERCEPTION_HEIGHT), interpolation=cv2.INTER_LINEAR)
        preview[edge_preview > 0] = (0, 255, 0)

        scale_x = Config.PERCEPTION_WIDTH / frame.shape[1]
        scale_y = Config.PERCEPTION_HEIGHT / frame.shape[0]
        scaled_roi = np.array(
            [[(int(x * scale_x), int(y * scale_y)) for x, y in roi_poly]],
            np.int32,
        )
        cv2.polylines(preview, scaled_roi, True, (0, 180, 255), 1)
        return preview


class DebugHUD:
    def __init__(self):
        self.font = pygame.font.Font(None, 24)

    def draw(self, screen, vehicle: VehicleController, fps: float, render_ms: float, cv_ms: float):
        mode = "MANUAL" if vehicle.override_fade > 0.3 else "AUTO"
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

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    cv2.destroyAllWindows()
                    return
            keys = pygame.key.get_pressed()

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
