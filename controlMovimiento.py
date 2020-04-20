#!/usr/bin/env python

"""
# Este código se encarga de controlar el movimiento del Rover, 
# controlado desde un joystick (control de xbox) o mediante visión
# (utilizando el código "vision.py"). Da como prioridad el control 
# al joystick en todo momento, por seguridad; además de contar con
# un botón de paro de emergencia físico en el Rover que corta la
# alimentación de los motores. 
"""

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
myJoy=Joy()
myJoy.buttons=[0,0]
myJoy.axes=[0,0,0,0,0]
radio=Int32()

def callback(data):
    global myJoy
    myJoy=data

def callback_1(data):
    global radio
    radio=data

def talker():
    global myJoy
    rospy.init_node('motor_joystick', anonymous=True)
    ##rospy.init_node('camara', anonymous=True)
    pub_wr = rospy.Publisher('arduino/wr', Int32, queue_size=10)
    pub_wl = rospy.Publisher('arduino/wl', Int32, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.Subscriber("radio", Int32, callback_1)
    rate = rospy.Rate(100) # 10hz
    wl = Int32()
    wr = Int32()
    state = bool()
    state = False


    while not rospy.is_shutdown():

        if myJoy.buttons[1] == 1:
            state=not state
        

        if not state:               
            wl=(myJoy.axes[1])*255
            wr=(myJoy.axes[4])*255  
            pub_wr.publish(wr) 
            pub_wl.publish(wl)
        else: 
           if radio.data<=40:
                
                wl=200
                wr=200  
                pub_wr.publish(wr) 
                pub_wl.publish(wl)
           elif radio.data<=150 and radio.data>=60:
               
                wl=-200
                wr=-200  
                pub_wr.publish(wr) 
                pub_wl.publish(wl)
           else:
                wl=0
                wr=0  
                pub_wr.publish(wr) 
                pub_wl.publish(wl)
                
                

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

