#!/usr/bin/env python3

import os
import keyboard

# set hover position
def go_to_center():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 1 --yaw -0')
    print('go to center')

def start():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 1 --vel 1 0 0 --yaw -0')
    print('start')
    
def stop():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 1 --vel 2 0 0 --yaw -0')
    print('stop')
    
def kill():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --vel 3 0 0 --yaw -0')
    print('kill')
    
def nn_init():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --vel 4 0 0 --yaw -0')
    print('nn_init')
    
def nn_activate():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --vel 5 0 0 --yaw -0')
    print('nn_activate')
    
def recovery_mode():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --vel 6 0 0 --yaw -0')
    print('recovery_mode')
    
def set_speed(speed):
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --vel 0 {speed} 0 --yaw -0')
    print(f'set speed = {speed} m/s')
    

def land():
    stop()
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0 --yaw -0')
    print('land')
    
# EXTRA
def wp1():
    os.system(f'./setpoint_sender/setpointSender.py --pos 2 0 0.75 --yaw -0')
    print('wp1')
    
def wp2():
    os.system(f'./setpoint_sender/setpointSender.py --pos -2 0 0.75 --yaw -0')
    print('wp2')
    
def wp3():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 2 0.75 --yaw -0')
    print('wp3')
    
def wp4():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 -2 0.75 --yaw -0')
    print('wp4')
    
def wp5():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 3.5 --yaw -0')
    print('wp5')
    
def wp6():
    os.system(f'./setpoint_sender/setpointSender.py --pos 0 0 0.75 --yaw -180')
    print('wp6')

speed = 0.0
info = '''
current speed = {speed} m/s
0 = go to center
1 = start
2 = decrease speed by 0.5 m/s
3 = increase speed by 0.5 m/s
4 = stop
5 = land
6 = nn_init
7 = nn_activate
8 = recovery_mode
------------------
left arrow  = wp1
right arrow = wp2
up arrow    = wp3
down arrow  = wp4
space bar   = wp5
backspace   = wp6
------------------
9 = kill (doesnt work)
'''

print(info.format(speed=speed))

def keyboard_callback(event):
    global speed
    if keyboard.is_pressed('0'):
        go_to_center()
    elif keyboard.is_pressed('1'):
        start()
    elif keyboard.is_pressed('2'):
        speed -= 0.5
        set_speed(speed)
    elif keyboard.is_pressed('3'):
        speed += 0.5
        set_speed(speed)
    elif keyboard.is_pressed('4'):
        stop()
    elif keyboard.is_pressed('5'):
        land()
    elif keyboard.is_pressed('6'):
        nn_init()
    elif keyboard.is_pressed('7'):
        nn_activate()
    elif keyboard.is_pressed('8'):
        recovery_mode()
    elif keyboard.is_pressed("left arrow"):
        wp1()
    elif keyboard.is_pressed("right arrow"):
        wp2()
    elif keyboard.is_pressed("up arrow"):
        wp3()
    elif keyboard.is_pressed("down arrow"):
        wp4()
    elif keyboard.is_pressed('space'):
        wp5()
    elif keyboard.is_pressed('backspace'):
        wp6()
    elif keyboard.is_pressed('9'):
        kill()
    print(info.format(speed=speed))
        
keyboard.on_press(keyboard_callback)

while True:
    # wait
    pass

# while True:
#     # key = input()
#     if keyboard.is_pressed('1'):
#         start()
#         print(info.format(speed=speed))
#     elif keyboard.is_pressed('2'):
#         speed -= 0.5
#         set_speed(speed)
#         print(info.format(speed=speed))
#     elif keyboard.is_pressed('3'):
#         speed += 0.5
#         set_speed(speed)
#         print(info.format(speed=speed))
#     elif keyboard.is_pressed('4'):
#         stop()
#     elif keyboard.is_pressed('5'):
#         land()
#     elif keyboard.is_pressed('0'):
#         disarm()
#         break