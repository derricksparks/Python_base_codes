import sys
import pygame
import math
import numpy as np

pygame.font.init()
font = pygame.font.SysFont('Times New Roman', 20)


def drawText(screen, s, x, y, color=(0, 0, 0)):
    surf = font.render(s, True, color)
    screen.blit(surf, (x, y))


sz = (800, 900)


def rot(v, ang):
    s, c = math.sin(ang), math.cos(ang)
    x = v[0] * c - v[1] * s
    y = v[0] * s + v[1] * c
    return [x, y]


def rotArr(vv, ang):
    return [rot(v, ang) for v in vv]


def dist(p1, p2):
    return np.linalg.norm(np.subtract(p1, p2))


def drawRotRect(screen, color, pc, w, h, ang, width=2):
    pts = [
        [-w / 2, -h / 2],
        [w / 2, -h / 2],
        [-w / 2, h / 2],
        [w / 2, h / 2]
    ]
    pts = rotArr(pts, ang)
    pts = np.add(pts, pc)
    pygame.draw.polygon(screen, color, pts, width)


class Robot:
    def __init__(self, x, y, alpha=0):
        self.x = x
        self.y = y
        self.alpha = alpha
        self.L = 70        # wheelbase
        self.W = 40        # width
        self.steer = 0     # current steering angle (rad)
        self.speed = 80    # constant forward speed
        self.traj = []
        self.max_steer = 0.6  # ~34 degrees max steering angle

    def getpos(self):
        return [self.x, self.y]

    def draw(self, screen):
        p = np.array(self.getpos())
        drawRotRect(screen, (0, 0, 0), p, self.L, self.W, self.alpha)

        # Draw wheels
        wheel_offset = self.L * 0.4
        wheel_positions = [
            [-wheel_offset, -self.W / 2],
            [-wheel_offset, self.W / 2],
            [wheel_offset, -self.W / 2],
            [wheel_offset, self.W / 2]
        ]
        for wp in wheel_positions:
            wp_rot = rot(wp, self.alpha)
            abs_pos = p + wp_rot
            # Front wheels turn with steering
            wheel_ang = self.alpha + self.steer if wp[0] > 0 else self.alpha
            drawRotRect(screen, (50, 50, 50), abs_pos, 20, 10, wheel_ang, width=0)

        # Draw trajectory
        if len(self.traj) > 1:
            pygame.draw.lines(screen, (0, 100, 255), False, self.traj, 2)

    def pure_pursuit(self, target_pos, lookahead=80):
        """Pure pursuit controller - steers toward a point ahead on the path"""
        robot_pos = np.array(self.getpos())
        to_target = target_pos - robot_pos
        distance = np.linalg.norm(to_target)

        if distance < 15:
            self.steer = 0
            return

        # Look-ahead point: extend in direction of target
        direction = to_target / distance
        lookahead_point = robot_pos + direction * lookahead

        # Transform lookahead point to robot-local coordinates
        dx = lookahead_point[0] - self.x
        dy = lookahead_point[1] - self.y
        local_x = dx * math.cos(-self.alpha) - dy * math.sin(-self.alpha)
        local_y = dx * math.sin(-self.alpha) + dy * math.cos(-self.alpha)

        # Curvature = 2 * y / (x^2 + y^2)
        if abs(local_x) < 1e-3:
            curvature = 10 * np.sign(local_y)
        else:
            curvature = 2 * local_y / (local_x**2 + local_y**2)

        # Desired steering angle = atan(curvature * L)
        desired_steer = math.atan2(2 * curvature * self.L, 1.0)

        # Limit and smooth steering
        desired_steer = np.clip(desired_steer, -self.max_steer, self.max_steer)
        steer_rate = 2.0
        self.steer += np.clip(desired_steer - self.steer, -steer_rate, steer_rate)

    def sim(self, dt):
        # Add current position to trail
        if not self.traj or dist(self.getpos(), self.traj[-1]) > 8:
            self.traj.append(self.getpos())

        # Kinematic bicycle model
        if abs(self.steer) > 0.01:
            R = self.L / math.tan(abs(self.steer))
            omega = self.speed / R
            self.alpha += omega * dt * np.sign(self.steer)

        dx = self.speed * math.cos(self.alpha) * dt
        dy = self.speed * math.sin(self.alpha) * dt
        self.x += dx
        self.y += dy


# Moving target that orbits in a circle
class MovingTarget:
    def __init__(self, color, phase, cx=400, cy=450, radius=200, speed=0.5):
        self.cx = cx
        self.cy = cy
        self.radius = radius
        self.angular_speed = speed
        self.color = color
        self.phase = phase
        self.angle = 0

    def update(self, dt):
        self.angle += self.angular_speed * dt

    def get_pos(self):
        x = self.cx + self.radius * math.cos(self.angle)
        y = self.cy + self.radius * math.sin(self.angle)
        return np.array([x, y])

    def draw(self, screen):
        pos = self.get_pos()
        pygame.draw.circle(screen, (255, 0, 0), pos.astype(int), 14)
        pygame.draw.circle(screen, (*self.color[:3], 100), pos.astype(int), 40, 4)


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode(sz)
    pygame.display.set_caption("Robot Pursuing Moving Target - Pure Pursuit")
    clock = pygame.time.Clock()
    fps = 60

    robot = Robot(400, 600, 0)
    targets = [
        MovingTarget(cx=500, cy=450, radius=180, speed=0.7, color=(255, 0, 0), phase=0),
        MovingTarget(cx=500, cy=450, radius=220, speed=0.45, color=(0, 180, 250), phase=2.0)
    ]

    time = 0
    running = True

    while running:
        dt = clock.tick(fps) / 1000.0
        time += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update moving target
        for t in targets:
            t.update(dt)

        # Robot pursues the moving target/Find the closest target
        robot_pos = np.array(robot.getpos())
        distances = [np.linalg.norm(t.get_pos() - robot_pos) for t in targets]
        closest_target = targets[np.argmin(distances)]

        # Robot chases only the closest one
        robot.pure_pursuit(closest_target.get_pos(), lookahead=110)
        robot.sim(dt)

        # Rendering
        screen.fill((240, 240, 255))

        for t in targets:
            t.draw(screen)

        robot.draw(screen)

        # Optional: show line to the CURRENT (closest) target only
        pygame.draw.line(screen, (255, 150, 0),
                         robot.getpos(), closest_target.get_pos(), 3)

        drawText(screen, f"Time: {time:.1f}s", 10, 10)
        drawText(screen, "Robot chases the NEAREST target", 10, 40, (100, 100, 100))
        drawText(screen, "Red & Blue circles = moving targets", 10, 70)
        pygame.display.flip()

    pygame.quit()
    sys.exit()
