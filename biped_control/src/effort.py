#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
import csv
import signal
import numpy as np
import matplotlib.pyplot as plt

joint_name = ['BFZ_SX', 'BFY_SX', 'BFX_SX', 'G_SX', 'TPX_SX', 'TPY_SX', 'BFZ_DX', 'BFY_DX', 'BFX_DX', 'G_DX', 'TPX_DX', 'TPY_DX']

global mux
global BFZ_SX, BFY_SX, BFX_SX, G_SX, TPX_SX, TPY_SX
global BFZ_DX, BFY_DX, BFX_DX, G_DX, TPX_DX, TPY_DX

mux = False

BFZ_SX = []
BFY_SX = []
BFX_SX = []
G_SX   = []
TPX_SX = []
TPY_SX = []
BFZ_DX = []
BFY_DX = []
BFX_DX = []
G_DX   = []
TPX_DX = []
TPY_DX = []


def callback(msg):

    global mux
    if mux == True:
        global BFZ_SX, BFY_SX, BFX_SX, G_SX, TPX_SX, TPY_SX
        global BFZ_DX, BFY_DX, BFX_DX, G_DX, TPX_DX, TPY_DX

	BFZ_SX.append(abs(msg.effort[0]))
	BFY_SX.append(abs(msg.effort[1]))
	BFX_SX.append(abs(msg.effort[2]))
	G_SX.append(abs(msg.effort[3]))
	TPX_SX.append(abs(msg.effort[4]))
	TPY_SX.append(abs(msg.effort[5]))
	BFZ_DX.append(abs(msg.effort[6]))
	BFY_DX.append(abs(msg.effort[7]))
	BFX_DX.append(abs(msg.effort[8]))
	G_DX.append(abs(msg.effort[9]))
	TPY_DX.append(abs(msg.effort[10]))
	TPX_DX.append(abs(msg.effort[11]))
    	mux = False

def plot_effort():
    global BFZ_SX, BFY_SX, BFX_SX, G_SX, TPX_SX, TPY_SX
    global BFZ_DX, BFY_DX, BFX_DX, G_DX, TPX_DX, TPY_DX

    plt.subplot(2, 6, 1)
    plt.plot(BFZ_SX)	
    plt.title("BFZ_SX")

    plt.subplot(2, 6, 2)
    plt.plot(BFX_SX)
    plt.title("BFX_SX")

    plt.subplot(2, 6, 3)
    plt.plot(BFY_SX)
    plt.title("BFY_SX")
	
    plt.subplot(2, 6, 4)
    plt.plot(G_SX)
    plt.title("G_SX")

    plt.subplot(2, 6, 5)
    plt.plot(TPY_SX)
    plt.title("TPY_SX")
	
    plt.subplot(2, 6, 6)
    plt.plot(TPX_SX)
    plt.title("TPX_SX")

    plt.subplot(2, 6, 7)
    plt.plot(BFZ_DX)	
    plt.title("BFZ_DX")

    plt.subplot(2, 6, 8)
    plt.plot(BFX_DX)
    plt.title("BFX_DX")

    plt.subplot(2, 6, 9)
    plt.plot(BFY_DX)
    plt.title("BFY_DX")
	
    plt.subplot(2, 6, 10)
    plt.plot(G_DX)
    plt.title("G_DX")

    plt.subplot(2, 6, 11)
    plt.plot(TPY_DX)	
    plt.title("TPY_DX")

    plt.subplot(2, 6, 12)
    plt.plot(TPX_DX)
    plt.title("TPX_DX")
    plt.show()

def main():
    rospy.init_node("effort_save")
    while not rospy.is_shutdown(): 
	global mux
	mux = True
	#print mux
    	rospy.Subscriber('/biped_sensor/joint_states', JointState, callback)
	time.sleep(0.25)
    
        


if __name__ == '__main__':
    main()
    plot_effort()





