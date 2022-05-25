#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 파이썬3 용!

from operator import truediv
from pynput import keyboard
import sys
import select
import termios
import tty
import threading
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

release_flag = False

"""
터미널에서 키 입력시 창이 지저분해지는 것을 방지하기 위해 getKey() 함수로 글자들을 먹어주고 서브 쓰레드에서 keyboard 입력을 받아주고 
현재 누르고 있는 키들을 set에 보관한다.

pip3 install pynput  로 keyboard 모듈을 설치.
"""

def on_press(key):
    try:
        if key not in pressed_keys:
            # print('Alphanumeric key pressed: {0} '.format(key.char)) # 현재 내 코드에서 이거 쓰면 이상해짐..
            # print(type(str(key)))
            pressed_keys.add(str(key)) # 특수문자 확인 위해 str로 변환
            
    except AttributeError:
        if key not in pressed_keys:
            print('special key pressed: {0}'.format(key))
            
            pressed_keys.add(key)

    
    except Exception as e:
        print(e)


def on_release(key):
    # print('Key released: {0}'.format(key))
    pressed_keys.discard(str(key))
    
    # r키 뗄때 작동하도록
    if str(key) == "'r'": 
        switch_fwdrev()
        
    if str(key) == "'c'": 
        switch_easy_ctrl_mode()
    
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def getKey(key_timeout):
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
    return key


# Collect events until released
def Listener():
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
    
        
class Listener_class(threading.Thread):
    """daemon은 메인 함수가 종료되면 서브 스레드도 자동으로 종료되도록 하는 기능이다."""
    def __init__(self, on_press, on_release, daemon):
        super().__init__(daemon=daemon)
        
        self.on_press = on_press
        self.on_release = on_release
        print('start')
        
        
    def run(self):
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            
            listener.join()
        
def check_keys(pressed_keys):
    if pressed_keys: # 어떤 키를 눌렀을 때
        message = ''


        # 전후진 
        if 'Key.space' in pressed_keys:
            accel_pub.publish("brake")
            message += "brake "           
        
        elif "'w'" in pressed_keys and not "'s'" in pressed_keys:
            # print("go foward") # 차후 실제 전진 명령으로 수정
            accel_pub.publish("foward")
            message += "foward "
            
        elif "'s'" in pressed_keys and not "'w'" in pressed_keys:
            # print("brake") # 차후 실제 후진 명령으로 수정
            accel_pub.publish("backward")
            message += "backward "
            
        elif 'Key.space' in pressed_keys:
            accel_pub.publish("brake")
            message += "brake "    

        else:
            accel_pub.publish("0")            
            
            
        # 좌우 방향
        if "'a'" in pressed_keys and not "'d'" in pressed_keys:
            steering_pub.publish("left")
            message += "left "
            
        elif "'d'" in pressed_keys and not "'a'" in pressed_keys:
            steering_pub.publish("right")
            message += "right "
            
        elif not "'d'" in pressed_keys and not "'a'" in pressed_keys:
            steering_pub.publish("0")

        else:
            steering_pub.publish("0")


        # r,c 키 뗄때(on_release) 작동하도록 바꿨음
          
            
        print(message)
            
    else: # 아무 키도 누르지 않았을 때
        accel_pub.publish("0")
        steering_pub.publish("0")
    

def switch_fwdrev():
    empty = Empty()
    fwdrev_pub.publish(empty)
    print("reversed")
    global release_flag
    release_flag = True
    
    
def switch_easy_ctrl_mode():
    empty = Empty()
    easy_ctrl_mode_pub.publish(empty)
    
    print("switch easy control mode")
    global release_flag
    release_flag = True
    
            
            
        
if __name__ == '__main__':
    try:
        pressed_keys = set()
        
        rospy.init_node('keyboard_ctrl_node')
        steering_pub = rospy.Publisher('steering_wheel_control_topic', String, queue_size=10)
        accel_pub = rospy.Publisher('accel_control_topic', String, queue_size=10)
        fwdrev_pub = rospy.Publisher('fwdrev_topic', Empty, queue_size=10)
        easy_ctrl_mode_pub = rospy.Publisher('easy_ctrl_mode_topic', Empty, queue_size=10)

        r = rospy.Rate(60) # getKey가 터미널에서 key입력값 빨리 먹으라고, 안그럼 터미널이 지저분해짐.
        
        l = Listener_class(on_press=on_press, on_release=on_release, daemon=True)
        l.start()
        
        while not rospy.is_shutdown():
            # print(pressed_keys)
            
            check_keys(pressed_keys)
            
            r.sleep()

            if (getKey(0.1) == '\x03'): # \x03 은 ctrl c 키
                accel_pub.publish("brake") # 종료 시 브레이크
                break
            
            if (release_flag == True): # r,c키 누른 다음 나오는 reversed 다음줄이 탭이 걸려있어서 없애는 역할
                print("")
                release_flag = False
           
            
        print('main tread end')
        
    except rospy.ROSInterruptException:
        pass
        
