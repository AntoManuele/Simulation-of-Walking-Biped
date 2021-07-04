#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Bool, Int16
from gazebo_msgs.msg import ContactState
from gazebo_msgs.msg import ContactsState
from control_msgs.msg import JointControllerState
import time
import math
import numpy as np
from rospy.numpy_msg import numpy_msg


#1 2 5 9
joint_position = JointControllerState()
sensor_dx1 = ContactState()
sensor_dx2 = ContactState()
sensor_dx3 = ContactState()
sensor_dx4 = ContactState()
###########################
sensor_sx1 = ContactState()
sensor_sx2 = ContactState()
sensor_sx3 = ContactState()
sensor_sx4 = ContactState()
ack_dx = rospy.Publisher("ack_dx", Int16, queue_size=16)
ack_sx = rospy.Publisher("ack_sx", Int16, queue_size=16)

###########################################################

global mutex_dx_1, mutex_dx_2, mutex_dx_3, mutex_dx_4
global mutex_sx_1, mutex_sx_2, mutex_sx_3, mutex_sx_4, err_R, err_L
err_R = 0
err_L = 0
mutex_dx_1 = False
mutex_dx_2 = False
mutex_dx_3 = False
mutex_dx_4 = False
mutex_sx_1 = False
mutex_sx_2 = False
mutex_sx_3 = False
mutex_sx_4 = False
Request = False

#############################################################

def callback(data):
    global Request 
    Request = data.data

#############################################################

def callback_Dx1(msg):
    global sensor_dx1, mutex_dx_1, err_R
    if mutex_dx_1 == True:
        try:
            sensor_dx1.collision2_name = msg.states[0].collision2_name
	    err_R = err_R + 1
        except: 
	    sensor_dx1.collision2_name = 'none'
    mutex_dx_1 = False

def callback_Dx2(msg):
    global sensor_dx2, mutex_dx_2, err_R
    if mutex_dx_2 == True:
        try:
            sensor_dx2.collision2_name = msg.states[0].collision2_name
	    err_R = err_R + 2
        except: 
	    sensor_dx2.collision2_name = 'none'
	mutex_dx_2 = False

def callback_Dx3(msg):
    global sensor_dx3, mutex_dx_3, err_R
    if mutex_dx_3 == True:
        try:
            sensor_dx3.collision2_name = msg.states[0].collision2_name
	    err_R = err_R + 5
        except: 
	    sensor_dx3.collision2_name = 'none'
    mutex_dx_3 = False

def callback_Dx4(msg):
    global sensor_dx4, mutex_dx_4, err_R
    if mutex_dx_4 == True:
        try:
            sensor_dx4.collision2_name = msg.states[0].collision2_name
	    err_R = err_R + 9
        except: 
	    sensor_dx4.collision2_name = 'none'
    mutex_dx_4 = False

################################################################

def callback_Sx1(msg):
    global sensor_sx1, mutex_sx_1, err_L
    if mutex_sx_1 == True:
        try:
            sensor_sx1.collision2_name = msg.states[0].collision2_name
	    err_L = err_L - 1
        except: 
	    sensor_sx1.collision2_name = 'none'
    mutex_sx_1 = False

def callback_Sx2(msg):
    global sensor_sx2, mutex_sx_2, err_L
    if mutex_sx_2 == True:
        try:
            sensor_sx2.collision2_name = msg.states[0].collision2_name
	    err_L = err_L - 2
        except: 
	    sensor_sx2.collision2_name = 'none'
    mutex_sx_2 = False

def callback_Sx3(msg):
    global sensor_sx3, mutex_sx_3, err_L
    if mutex_sx_3 == True:
        try:
            sensor_sx3.collision2_name = msg.states[0].collision2_name
	    err_L = err_L - 5
        except: 
	    sensor_sx3.collision2_name = 'none'
    mutex_sx_3 = False

def callback_Sx4(msg):
    global sensor_sx4, mutex_sx_4, err_L
    if mutex_sx_4 == True:
        try:
            sensor_sx4.collision2_name = msg.states[0].collision2_name
	    err_L = err_L - 9
        except: 
	    sensor_sx4.collision2_name = 'none'
    mutex_sx_4 = False



def sendcode():
    global err_R, err_L
    print 'err_R----->', err_R
    print 'err_L----->', err_L 
    print '-----------------'
    ack_dx.publish(err_R) 
    ack_sx.publish(err_L)   

###########################################################################

def main():
    rospy.init_node("sensor")
    print 'initialize sensor node'
    while not rospy.is_shutdown(): 
   	rospy.Subscriber('/request', Bool, callback)
        global Request
	global mutex_dx_1, mutex_dx_2, mutex_dx_3, mutex_dx_4
	global mutex_sx_1, mutex_sx_2, mutex_sx_3, mutex_sx_4    

	if Request == True:
	    global err_R, err_L
	    err_R = 0
	    err_L = 0
	    mutex_dx_1 = True
            rospy.Subscriber('/sensore_contact_dx1', ContactsState, callback_Dx1)
	    time.sleep(0.1)
	    mutex_dx_2 = True
            rospy.Subscriber('/sensore_contact_dx2', ContactsState, callback_Dx2)
	    time.sleep(0.1)
	    mutex_dx_3 = True
            rospy.Subscriber('/sensore_contact_dx3', ContactsState, callback_Dx3)
	    time.sleep(0.1)
	    mutex_dx_4 = True
            rospy.Subscriber('/sensore_contact_dx4', ContactsState, callback_Dx4)
	    time.sleep(0.1)


	    mutex_sx_1 = True
	    rospy.Subscriber('/sensore_contact_sx1', ContactsState, callback_Sx1)
	    mutex_sx_2 = True
            rospy.Subscriber('/sensore_contact_sx2', ContactsState, callback_Sx2)
	    mutex_sx_3 = True
            rospy.Subscriber('/sensore_contact_sx3', ContactsState, callback_Sx3)
	    mutex_sx_4 = True
            rospy.Subscriber('/sensore_contact_sx4', ContactsState, callback_Sx4)
	    
	    rospy.sleep(0.5)
	    sendcode()


	    Request = False

        time.sleep(1)

if __name__ == '__main__':
    main()



