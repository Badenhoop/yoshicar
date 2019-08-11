#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import pygame
import sys
from pexpect import pxssh
from pygame.locals import *
import threading
import sys
import numpy as np
from time import sleep

def center(src, dest):
    src.centerx = dest.centerx
    src.centery = dest.centery

class Button:
    def __init__(self, rect, label, onPressed=None, onReleased=None):
        self.rect = rect
        self.label = label
        self.onPressed = onPressed
        self.onReleased = onReleased
        self.pressed = False

    def press(self):
        if not self.pressed and self.onPressed is not None:
            self.onPressed()
        self.pressed = True

    def release(self):
        if self.pressed and self.onReleased is not None:
            self.onReleased()
        self.pressed = False

    def draw(self, surf):
        # fill
        if not self.pressed:
            pygame.draw.rect(surf, (150, 150, 150), self.rect)
        else:
            pygame.draw.rect(surf, (255, 100, 100), self.rect)
        # frame
        pygame.draw.rect(surf, (10, 10, 10), self.rect, 1)
        # label
        font = pygame.font.Font(None, 36)
        text = font.render(self.label, True, (10, 10, 10))
        text_rect = text.get_rect()
        center(text_rect, self.rect)
        surf.blit(text, text_rect)
        
def create_joystick():
    joystick = None
    try:
        joystick = pygame.joystick.Joystick(0) # create a joystick instance
        joystick.init() # init instance
        print("Enabled joystick: " + joystick.get_name())
    except pygame.error:
        print("No joystick found.")
    return joystick

def main():
    rospy.init_node('remote_control_gui')
    motor_pub = rospy.Publisher('motor', Float64, queue_size=10)
    servo_pub = rospy.Publisher('servo', Float64, queue_size=10)

    FPS = 60
    pygame.init()
    pygame.joystick.init()
    fpsClock = pygame.time.Clock()

    BACKGROUND_COLOR = (255, 255, 255)
    SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill(BACKGROUND_COLOR)
    background.blit(background, (0,0))

    clock = pygame.time.Clock()
    pygame.key.set_repeat(1, 40)

    BUTTON_WIDTH = 200
    BUTTON_HEIGHT = 100
    PADDING = 10
    buttonsSurface = pygame.Surface((3 * BUTTON_WIDTH + 2 * PADDING, 2 * BUTTON_HEIGHT + PADDING))
    buttonsSurface = buttonsSurface.convert()
    buttonsSurface.fill(BACKGROUND_COLOR)

    speed_modes_forward = [0.2, 0.3, 0.4]
    speed_modes_backward = [-0.2, -0.3, -0.4]
    curr_speed_mode = 0
    zero_speed = 0
    max_angle = 1.0
    zero_angle = 0

    forwards = Button(
        pygame.Rect(1 * (BUTTON_WIDTH + PADDING), 0 * (BUTTON_HEIGHT + PADDING), BUTTON_WIDTH, BUTTON_HEIGHT),
        "FORWARDS",
        onPressed=lambda: motor_pub.publish(speed_modes_forward[curr_speed_mode]),
        onReleased=lambda: motor_pub.publish(zero_speed))

    backwards = Button(
        pygame.Rect(1 * (BUTTON_WIDTH + PADDING), 1 * (BUTTON_HEIGHT + PADDING), BUTTON_WIDTH, BUTTON_HEIGHT),
        "BACKWARDS",
        onPressed=lambda: motor_pub.publish(speed_modes_backward[curr_speed_mode]),
        onReleased=lambda: motor_pub.publish(zero_speed))

    left = Button(
        pygame.Rect(0 * (BUTTON_WIDTH + PADDING), 1 * (BUTTON_HEIGHT + PADDING), BUTTON_WIDTH, BUTTON_HEIGHT),
        "LEFT",
        onPressed=lambda: servo_pub.publish(max_angle),
        onReleased=lambda: servo_pub.publish(zero_angle))

    right = Button(
        pygame.Rect(2 * (BUTTON_WIDTH + PADDING), 1 * (BUTTON_HEIGHT + PADDING), BUTTON_WIDTH, BUTTON_HEIGHT),
        "RIGHT",
        onPressed=lambda: servo_pub.publish(-max_angle),
        onReleased=lambda: servo_pub.publish(zero_angle))

    buttons = [forwards, backwards, left, right]
    buttons_rect = buttonsSurface.get_rect()
    center(buttons_rect, background.get_rect())

    joystick = create_joystick()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                running = False
                break
            elif event.type == KEYDOWN:
                if event.key == K_UP:
                    forwards.press()
                elif event.key == K_DOWN:
                    backwards.press()
                elif event.key == K_LEFT:
                    left.press()
                elif event.key == K_RIGHT:
                    right.press()
                elif event.key == K_1:
                    curr_speed_mode = 0
                elif event.key == K_2:
                    curr_speed_mode = 1
                elif event.key == K_3:
                    curr_speed_mode = 2
            elif event.type == KEYUP:
                if event.key == K_UP:
                    forwards.release()
                elif event.key == K_DOWN:
                    backwards.release()
                elif event.key == K_LEFT:
                    left.release()
                elif event.key == K_RIGHT:
                    right.release()
            elif event.type == JOYAXISMOTION:
                x = joystick.get_axis(0)
                y = joystick.get_axis(4)
                angle = min(max(x * max_angle, -max_angle), max_angle)
                speed = min(max(-y * MAX_SPEED, MIN_SPEED), MAX_SPEED)
                servo_pub.publish(angle)
                motor_pub.publish(speed)

        background.fill(BACKGROUND_COLOR)

        for button in buttons:
            button.draw(buttonsSurface)

        background.blit(buttonsSurface, buttons_rect)
        screen.blit(background, (0, 0))
        pygame.display.update()
        fpsClock.tick(FPS)

if __name__ == '__main__':
    main()
