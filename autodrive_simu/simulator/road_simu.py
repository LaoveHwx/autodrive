import pygame
import cv2
import numpy as np
from perception.lane_detector import detect_lane

WIDTH = 800
HEIGHT = 600
ROAD_WIDTH = 400


def draw_road(screen, offset):

    road_left = WIDTH//2 - ROAD_WIDTH//2

    pygame.draw.rect(screen,(60,60,60),(road_left,0,ROAD_WIDTH,HEIGHT))

    lane_x1 = road_left + ROAD_WIDTH//3
    lane_x2 = road_left + ROAD_WIDTH*2//3

    for y in range(-40,HEIGHT,40):

        pygame.draw.line(
            screen,(255,255,255),
            (lane_x1,y+offset),(lane_x1,y+20+offset),4
        )

        pygame.draw.line(
            screen,(255,255,255),
            (lane_x2,y+offset),(lane_x2,y+20+offset),4
        )


def run_simulator():

    pygame.init()

    screen = pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("AutoDrive Simulator")

    clock = pygame.time.Clock()

    car_x = WIDTH//2
    car_y = HEIGHT-120
    car_w = 40
    car_h = 60

    speed = 6
    road_offset = 0

    running=True

    while running:

        clock.tick(60)

        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running=False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_a]:
            car_x -= speed

        if keys[pygame.K_d]:
            car_x += speed

        road_offset += 8
        road_offset %= 40

        screen.fill((20,20,20))

        draw_road(screen,road_offset)

        pygame.draw.rect(
            screen,
            (255,165,0),
            (car_x,car_y,car_w,car_h)
        )

        pygame.display.update()

        # =========================
        # pygame → opencv frame
        # =========================

        frame = pygame.surfarray.array3d(screen)
        frame = np.transpose(frame,(1,0,2))
        frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)

        # =========================
        # lane detection
        # =========================

        roi,left_lines,right_lines = detect_lane(frame)

        # 可视化检测到的线
        for l in left_lines:
            x1,y1,x2,y2 = l
            cv2.line(frame,(x1,y1),(x2,y2),(255,0,0),2)

        for l in right_lines:
            x1,y1,x2,y2 = l
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)

        lane_center=None

        if len(left_lines)>0 and len(right_lines)>0:

            left_x = np.mean([(l[0]+l[2])/2 for l in left_lines])
            right_x = np.mean([(l[0]+l[2])/2 for l in right_lines])

            lane_center = int((left_x+right_x)/2)

            cv2.line(
                frame,
                (lane_center,0),
                (lane_center,HEIGHT),
                (0,255,0),
                3
            )

        # =========================
        # vehicle center
        # =========================

        vehicle_center = car_x + car_w//2

        cv2.line(
            frame,
            (vehicle_center,0),
            (vehicle_center,HEIGHT),
            (0,255,255),
            2
        )

        # =========================
        # offset
        # =========================

        if lane_center is not None:
            offset = vehicle_center - lane_center
        else:
            offset = 0

        cv2.putText(
            frame,
            f"Offset: {offset}",
            (30,50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,255),
            2
        )

        # =========================
        # display
        # =========================

        cv2.imshow("lane_detect",frame)
        cv2.waitKey(1)

    pygame.quit()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_simulator()