#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

msg = """
Control Your Robot!
------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0

    def publish_cmd_vel(self, x, th):
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.angular.z = th * self.turn
        self.publisher.publish(twist)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    teleop_keyboard = TeleopKeyboard()
    
    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][3]
                teleop_keyboard.publish_cmd_vel(x, th)
            elif key == '\x03':
                break
            
    except Exception as e:
        print(e)
    
    finally:
        teleop_keyboard.publish_cmd_vel(0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 