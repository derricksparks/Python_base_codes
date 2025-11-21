import sys
import pygame
import math
import numpy as np
import itertools

pygame.font.init()
font = pygame.font.SysFont('Times new roman', 20)


def drawText(screen, s, x, y):
    surf = font.render(s, True, (0, 0, 0))
    screen.blit(surf, (x, y))


sz = (800, 900)


def rot(v, ang):  # rotate a vector by angle
    s, c = math.sin(ang), math.cos(ang)
    x = v[0] * c - v[1] * s
    y = v[0] * s + v[1] * c
    return [x, y]


def limAng(ang):  # defining the range of the angle to remain in a range of pi and -pi

    while ang > math.pi:
        ang -= 2 * math.pi
    while ang <= -math.pi:
        ang += 2 * math.pi
    return ang


def rotArr(vv, ang):  # defining the rotating angle of an array of vectors (функция для поворота массива на угол)

    return [rot(v, ang) for v in vv]


def dist(p1, p2):  # defining the distance using points p1 and p2

    return np.linalg.norm(np.subtract(p1, p2))


# center point, width, height of the rectangle and angle of rotation of the rectangle
def drawRotRect(screen, color, pc, w, h, ang):

    pts = [
        [-w / 2, -h / 2],
        [w / 2, -h / 2],
        [-w / 2, h / 2],
        [w / 2, h / 2]
    ]

    pts = rotArr(pts, ang)
    pts = np.add(pts, pc)
    pygame.draw.polygon(screen, color, pts, 2)


class Robot:  # defining the class of the robot
    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        self.L = 70
        self.W = 40
        self.steer = 0
        self.speed = 0
        self.traj = []  # trajectory points/Trail of positions

    def getpos(self):
        return [self.x, self.y]

    def clear(self):
        self.traj = []
        self.vals1 = []
        self.vals2 = []

    def draw(self, screen):  # defining the drawing of the robot on the screen
        p = np.array(self.getpos())
        drawRotRect(screen, (0, 0, 0), p, self.L, self.W, self.alpha)

        dx = self.L / 3
        dy = self.W / 3
        # drawing the wheels of th robot as small rectangles(dx,dy)are the sizes of the wheels
        dd = [[-dx, -dy], [-dx, dy], [dx, -dy], [dx, dy]]
        dd = rotArr(dd, self.alpha)
        kRot = [0, 0, 1, 1]  # defines which wheels rotate on the robot

        for d, k, in zip(dd, kRot):
            drawRotRect(screen, (0, 0, 0), p + d, self.L / 5,
                        self.W / 5, self.alpha + k * self.steer)
            for i in range(len(self.traj) - 1):  # drawing the trail of trajectory/movement
                pygame.draw.line(screen, (0, 0, 255), self.traj[i], self.traj[i + 1], 1)

    def sim(self, dt):  # defining the moving forward of the robot based on the current orientation
        self.addedTrajPt = False
        delta = [self.speed * dt, 0]
        delta = rot(delta, self.alpha)
        self.x += delta[0]
        self.y += delta[1]

        if self.steer != 0:  # handling steering(car like turning)
            R = self.L / self.steer  # turning radius
            da = self.speed * dt / R  # change in the angle
            self.alpha += da

        p = self.getpos()
        if len(self.traj) == 0 or dist(
                p, self.traj[-1]) > 10:  # recording the trajectory points periodically
            self.traj.append(self.getpos())
            self.addedTrajPt = True

# This is a simple proportional controller that steers toward the target position.

    def goto(self, pos, dt):  # calculating the direction to the goal
        v = np.subtract(pos, self.getpos())
        aGoal = np.atan2(v[1], v[0])  # target angle
        da = limAng(aGoal - self.alpha)  # finding the shortest angle difference
        self.steer += 0.5 * da * dt  # adjusting steering towards the target
        maxsteer = 1
        if self.steer > maxsteer:
            self.steer = maxsteer
        if self.steer < -maxsteer:
            self.steer = -maxsteer  # liming the steering range
        self.speed = 50  # constant speed forward


if __name__ == "__main__":
    screen = pygame.display.set_mode(sz)
    timer = pygame.time.Clock()
    fps = 20

    robot = Robot(100, 100, 1)  # robot starts from (100,100) and facing an angle 1

    time = 0
    goal = [600, 400]  # setting the target position

    while True:  # handling the window close event
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                sys.exit(0)

        dt = 1 / fps  # fixed time for consistent simulation

        screen.fill((250, 250, 250))  # clear white screen

        robot.goto(goal, dt)  # calculate steering
        robot.sim(dt)  # update the position
        robot.draw(screen)  # Draw the robot

        pygame.draw.circle(screen, (250, 0, 0), goal, 5, 2)  # draw the target
        drawText(screen, f"Time={time:.3f}", 5, 5)  # display time

        pygame.display.flip()  # update the screen
        timer.tick(fps)  # maintain the framerate
        time += dt
