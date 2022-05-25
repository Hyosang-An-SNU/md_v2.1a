#!/usr/bin/env python3
# coding=utf-8
# license removed for brevity


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


def cmd_posi():
    pub_posi = rospy.Publisher('cmd_posi', Int32, queue_size=1)
    pub_tq_free = rospy.Publisher('tq_free_topic', String, queue_size=1)
    pub_reset = rospy.Publisher('reset_topic', String, queue_size=1)
    pub_current_limit = rospy.Publisher('current_limit_topic', Int32, queue_size=1)
    rospy.init_node('cmd_posi_node', anonymous=True)

    #set the loop rate
    rate = rospy.Rate(10)  # 10hz
    #keep publishing until a Ctrl-C is pressed

    while not rospy.is_shutdown():

        try:
            _input = input('Target Position : ')

            #tq_free ON/OFF 설정
            if(_input == "ON" or _input == "OFF"):
                pub_tq_free.publish("tq_free " + _input)
                print(" \n   --- tq_free " + _input + " ---")

            # positon reset
            elif (_input == "reset"):
                pub_reset.publish(_input)
                print(" \n   --- reset ---")

            # current limit 설정
            elif(_input == "current limit set"):
                print("\n   --- Current limit setting ---")
                while not rospy.is_shutdown():
                    try:
                        current_limit_input = input("current limit (0.1A) : ")
                        if(current_limit_input == "back"):
                            print("\n   --- Position Command mode ---")
                            break

                        pub_current_limit.publish(int(current_limit_input) + 1)

                    except:
                        print('\n--------- please input a valid value. ------------')


            #position(int) 값 주는 경우
            else:
                pub_posi.publish(int(_input))
                rate.sleep()

        except Exception as e:
            print(e)
            print('\n--------- please input a valid value. ------------\n')


if __name__ == '__main__':
    try:
        cmd_posi()
    except rospy.ROSInterruptException:
        pass
