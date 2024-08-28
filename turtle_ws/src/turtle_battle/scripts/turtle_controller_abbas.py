#!/usr/bin/env python3
import rospy
import subprocess
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import sys, select, termios, tty
import os
from pathlib import Path

#
file_path1 ="/home/marwan/turtle_controll_ws/src/controll/scripts/file.txt"
file_path2 = "/home/marwan/turtle_controll_ws/src/controll/scripts/file_start.txt"
start = 0
try:
    with open(file_path2, 'r') as op:
        start = int(op.read().strip())
except IOError as e:
    print(f"Failed to read file: {e}")
    sys.exit(1)
if start:
    print("game started")
    exit(0)
try:
    with open(file_path1, 'r') as op:
        turtle_number = int(op.read().strip())
except IOError as e:
    print(f"Failed to read file: {e}")
    sys.exit(1)

curr_turtle = 1
choose = True
new = False

print('Control an existing turtle? (Y/n)')
ans = input().strip().lower()

if ans == 'y':
    print('Enter turtle number:')
    try:
        input_tur = int(input())
        if 1 <= input_tur <= turtle_number:
            curr_turtle = input_tur
            choose = False
        else:
            print("Automatic selection")
            curr_turtle = turtle_number
            choose = False
    except ValueError:
        print("Invalid input. Using automatic selection.")
        curr_turtle = turtle_number
        choose = False

if choose:
    while choose:
        turtle_number += 1
        command_spawn = f"rosservice call /spawn '{{x: {turtle_number}.0, y: 1.0, theta: 0.0, name: \"turtle{turtle_number}\"}}'"
        command_setpen = f'/turtle{turtle_number}/set_pen'        
        try:
            subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
            rospy.ServiceProxy(command_setpen, SetPen)(0, 0, 0 , 0, 1)
            curr_turtle = turtle_number
            new = True
            try:
                with open(file_path1, 'w') as op:
                    op.write(f"{turtle_number}")
            except IOError as e:
                print(f"Failed to write file: {e}")
            choose = False
        except subprocess.CalledProcessError as e:
            turtle_number -= 1
            # print(f"Failed to spawn turtle: {e}")
            continue

# Terminal settings for non-blocking key press
oldsettings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, oldsettings)
    return key


def start(data):
    if data.data == 1 or data.data == 0 :
        try:
            with open(file_path2, 'w') as op:
                op.write(f"{data.data}")
        except IOError as e:
            print(f"Failed to write file: {e}")
# Initialize ROS node
node_name = f"turtle_controller{curr_turtle}"
rospy.init_node(node_name)
turtle_name = f"turtle{curr_turtle}"
attack_pub = rospy.Publisher('/turtle_attack', Int8, queue_size=10)
new_pub = rospy.Publisher('/new_turtle', Int8, queue_size=10)

cmd_vel_pub = rospy.Publisher(f"/{turtle_name}/cmd_vel", Twist, queue_size=10)
twist = Twist()

if new:
    # rospy.loginfo("Publishing new turtle number: %d", curr_turtle)
    new_pub.publish(curr_turtle)
    new = False

print(f'Control {turtle_name} using WASD, Q to attack.')

while True:
    key = get_key()
    if key == 'w':
        twist.linear.x = 1.0
        twist.angular.z = 0.0
    elif key == 's':
        twist.linear.x = -1.0
        twist.angular.z = 0.0
    elif key == 'a':
        twist.linear.x = 0.0
        twist.angular.z = 3.1416 / 2
    elif key == 'd':
        twist.linear.x = 0.0
        twist.angular.z = -3.1416 / 2
    elif key == 'q':
        attack_pub.publish(curr_turtle)
        print(f"{turtle_name} attacks!")
        continue
    elif key == '\x03':
        break
    else:
        continue
    start_sub = rospy.Subscriber('/game_started', Int8, start)
    cmd_vel_pub.publish(twist)
