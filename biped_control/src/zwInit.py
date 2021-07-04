#!/usr/bin/env python

import sys
import time
import numpy as np
import os
import rospy
from std_msgs.msg import Float64, String, Bool, Int16
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
import signal
import getpass
import operator
 
recieved = 0

def sigint_handler(signum, frame):
	global recieved
	recieved = 1
	print('Closing')
	exit(0)

signal.signal(signal.SIGINT, sigint_handler)
dirpath = os.getcwd()

username = getpass.getuser()
print('-----------------------------------')
print('USER: ' + username)
########################################################

global err_R, err_L 
global error_R, error_L
global mutex_dx_1, mutex_dx_2, mutex_dx_3, mutex_dx_4
global mutex_sx_1, mutex_sx_2, mutex_sx_3, mutex_sx_4
global mutex_correction
global position_X_DX, position_X_SX, position_Y_DX, position_Y_SX
global corr_mux, actual_position

mutex_correction = False
mutex_dx = False
mutex_sx = False

corr_dx_X = False
corr_dx_Y = False
corr_sx_X = False
corr_sx_Y = False


err_R = 0
err_L = 0
extension = '.csv'
delim = ','
request_Var = True
mutexSubscriber = True

 #Casi obiettivo

goal_cases = [7, 8, 10, 12, 0, 15, 16, 17] 

rospy.init_node("walk")
BFZ_SX 	= 	rospy.Publisher("biped_sensor/bacino_femore_sx_Z_position/command", Float64, queue_size=16)
BFY_SX 	=	rospy.Publisher("biped_sensor/bacino_femore_sx_Y_position/command", Float64, queue_size=16)
BFX_SX 	=	rospy.Publisher("biped_sensor/bacino_femore_sx_X_position/command", Float64, queue_size=16)
G_SX 	=	rospy.Publisher("biped_sensor/ginocchio_sx_position/command", Float64, queue_size=16)
TPX_SX 	=	rospy.Publisher("biped_sensor/tibia_piede_sx_X_position/command", Float64, queue_size=16)
TPY_SX 	=	rospy.Publisher("biped_sensor/tibia_piede_sx_Y_position/command", Float64, queue_size=16)
BFZ_DX 	=	rospy.Publisher("biped_sensor/bacino_femore_dx_Z_position/command", Float64, queue_size=16)
BFY_DX 	=	rospy.Publisher("biped_sensor/bacino_femore_dx_Y_position/command", Float64, queue_size=16)
BFX_DX 	=	rospy.Publisher("biped_sensor/bacino_femore_dx_X_position/command", Float64, queue_size=16)
G_DX 	=	rospy.Publisher("biped_sensor/ginocchio_dx_position/command", Float64, queue_size=16)
TPX_DX 	=	rospy.Publisher("biped_sensor/tibia_piede_dx_X_position/command", Float64, queue_size=16)
TPY_DX 	=	rospy.Publisher("biped_sensor/tibia_piede_dx_Y_position/command", Float64, queue_size=16)
Request =	rospy.Publisher("request", Bool, queue_size=8)
Request_Joint =	rospy.Publisher("request_joint", Bool, queue_size=8)

def sendFunc(qVar):
    BFZ_SX.publish(qVar[0])
    BFX_SX.publish(qVar[1])
    BFY_SX.publish(qVar[2])
    G_SX.publish(qVar[3])
    TPY_SX.publish(qVar[4])
    TPX_SX.publish(qVar[5])
    BFZ_DX.publish(qVar[6])
    BFX_DX.publish(qVar[7])
    BFY_DX.publish(qVar[8])
    G_DX.publish(qVar[9])
    TPY_DX.publish(qVar[10])
    TPX_DX.publish(qVar[11])


def sendCorrection(corr):
    print('correction')
    print(corr)
    BFZ_SX.publish(corr[5])
    BFX_SX.publish(corr[3])
    BFY_SX.publish(corr[4])
    G_SX.publish(corr[7])
    TPY_SX.publish(corr[11])
    TPX_SX.publish(corr[10])
    BFZ_DX.publish(corr[2])
    BFX_DX.publish(corr[0])
    BFY_DX.publish(corr[1])
    G_DX.publish(corr[6])
    TPY_DX.publish(corr[9])
    TPX_DX.publish(corr[8])

#############################################################
def callback_dx(data):
    global mutex_dx
    if mutex_dx == True:
        global err_R
        err_R = data.data
        mutex_dx = False

#############################################################

def callback_sx(data):
	global mutex_sx
	if mutex_sx == True:
		global err_L
		err_L = data.data
		mutex_sx = False

############################################################

def callback_position(msg):
	global corr_mux, actual_position
	if corr_mux == True:
		actual_position = JointState()
		actual_position.position = msg.position
		corr_mux = False

############################################################

# Funzioni che ripubblicano i valori letti dai giunti aggiungendo una piccola correzione 

def send_correction_dx_X(sign):
	global actual_position
	if sign == 'pos':
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0.0015, 0, 0, 0)
	else: 
		corr = (0, 0, 0, 0, 0, 0, 0, 0, -0.0015, 0, 0, 0)
	data = tuple(map(operator.add, actual_position.position, corr))	
	sendCorrection(data)


def send_correction_sx_X(sign):
	global actual_position
	if sign == 'pos':
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0015, 0)
	else: 
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0015, 0)
	data = tuple(map(operator.add, actual_position.position, corr))	
	sendCorrection(data)

def send_correction_dx_Y(sign):
	global actual_position
	if sign == 'pos':
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0015, 0, 0)
	else: 
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0015, 0, 0)
	data = tuple(map(operator.add, actual_position.position, corr))	
	sendCorrection(data)

def send_correction_sx_Y(sign):
	global actual_position
	if sign == 'pos':
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0015)
	else: 
		corr = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0015)
	data = tuple(map(operator.add, actual_position.position, corr))	
	sendCorrection(data)


############################################################

def viewFoot():	
	time.sleep(0.3)
	Request.publish(True)
	print('viewFoot()')
	global mutexSubscriber
	mutexSubscriber = True
	if viewFoot_DX() == True and viewFoot_SX() == True:
		return True
	return False

###################################################################################################################

def viewFoot_SX():
	print('check foot SX')	
	Request.publish(True)
	global mutex_sx
	time.sleep(0.2)
	mutex_sx = True
	rospy.Subscriber('/ack_sx', Int16, callback_sx)
	if error_function_SX() == True:
		return True
	else:
		return False

def viewFoot_DX():
	print('check foot DX')
	Request.publish(True)
	global mutex_dx
	time.sleep(0.2)
	mutex_dx = True
	rospy.Subscriber('/ack_dx', Int16, callback_dx)
	if error_function_DX() == True:
		return True
	else:
		return False

###################################################################################################################

def error_function_DX():
	global err_R
	err_R = abs(err_R)
	if err_R in goal_cases:
		return True
	else:
		correction_function_DX(err_R)
	return False


def error_function_SX():
	global err_L
	err_L = abs(err_L)
	if err_L in goal_cases:
		return True
	else:
		correction_function_SX(err_L)
	return False

#############################################################################################################################

def correction_function_DX(err_R):
    #print'Correction Function Right'
    print('correction_function_R')
    if err_R == 3:
        fix_Y('R', 'pos')
    elif err_R == 14:
        fix_Y('R', 'neg')
    elif err_R == 6:
        fix_X('R', 'pos')
    elif err_R == 11:
        fix_X('R', 'neg')
    elif err_R == 1:
        fix_Y('R', 'neg')
    elif err_R == 2:
        fix_Y('R', 'neg')
    elif err_R == 5:
        fix_Y('R', 'pos')
    elif err_R == 9:
        fix_Y('R', 'pos')

	
###########################################################################################################################

def correction_function_SX(err_L):
    #print'Correction Function Left'
    print('correction_function_L')
    if err_L == 3:
        fix_Y('L', 'pos')
    elif err_L == 14:
        fix_Y('L', 'neg')
    elif err_L == 6:
        fix_X('L', 'pos')
    elif err_L == 11:
        fix_X('L', 'neg')
    elif err_L == 1:
        fix_Y('L', 'neg')
    elif err_L == 2:
        fix_Y('L', 'neg')
    elif err_L == 5:
        fix_Y('L', 'pos')
    elif err_L == 9:
        fix_Y('L', 'pos')
	
###########################################################################################################################

 # leggo la posizione dei giunti

def fix_X(foot, sign):
	print('FIX X!	Foot ---> ', foot) 
	global corr_mux
	corr_mux = True
	rospy.Subscriber('/biped_sensor/joint_states', JointState, callback_position)
	rospy.sleep(0.1)
	if foot == 'R':
		send_correction_dx_X(sign)
	else:
		send_correction_sx_X(sign)


def fix_Y(foot, sign):
	print('FIX Y!	Foot ---> ', foot)
	global corr_mux
	corr_mux = True
	rospy.Subscriber('/biped_sensor/joint_states', JointState, callback_position)
	rospy.sleep(0.1)
	if foot == 'R':
		send_correction_dx_Y(sign)
	else:
		send_correction_sx_Y(sign)
	

#################################################################################################################


#signR = np.array([1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1])
signR = np.array([1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1])
signL = (-1)*signR

sign = {"L": signL, "R": signR}

right = 'R'
left = 'L'

nameDict = ['initialL','startL','changeRtoL','walkL','juncWL','finishL','resetL','initialR','startR','changeLtoR','walkR','juncWR','finishR','resetR']


#FUNZIONE CHE GENERA LA SEQUENZA DI INDICI 
def XcsvDICT(startVar, stepVar):   
    res = np.array([])
    count = 1

    #GENERO LA SEQUENZA COME SE MI DOVESSI MUOVERE DI SICURO CON LA SINISTRA POI IN CASO INVERTO 'L' E 'R'
    res = np.array(["initialL","startL", "changeRtoL"])
     
    while count < stepVar:
        res = np.append(res, [ "walkR", "juncWR", "changeLtoR" ], axis = 0)
        count = count + 1
        if count < stepVar:
            res = np.append(res, [ "walkL", "juncWL", "changeRtoL" ], axis = 0)
            count = count + 1
     
    if(stepVar % 2 == 0):
        res = np.append(res, [ "finishL", "resetL" ], axis = 0)
    else:
        res = np.append(res, [ "finishR", "resetR" ], axis = 0)

    #INVERTO 'L' E 'R' SE DEVO INIZIARE A MUOVERE CON LA DESTRA
    if startVar == right:
        res = [i.replace("L","T") for i in res]
        res = [i.replace("R","L") for i in res]
        res = np.array([i.replace("T","R") for i in res])
     
    print('-----------------------------------')
    print('GENERATED SEQUENCE')
    for i in res:
        print('    ' + i)    
    
    return res
        
    

  
def zeroposFunc():    
    print('-----------------------------------')
    print('START ZERO')
    zeroQ = np.full(12, 0.)
    for j in range(1,1000):
        sendFunc(zeroQ)
        time.sleep(0.001)
    print('DONE ZERO')
    print('-----------------------------------')
    time.sleep(0.5)


def moveFunc(indexVar):

    print(indexVar)
    print('-----------------------------------')
    
    #moveVar prende l'ultima lettera del nome del file per scegliere il vettore dei segni
    moveVar = indexVar[-1]
    
    #METTO IN CSV LA MATRICE PRESENTE ALL'INDICE indexVar
    CSV = CSVdict[indexVar]

    for i in range(0, np.shape(CSV)[0]):

        #COSTANTE PER DELAY CHE MODIFICO SE E' UN CHANGE       
        const = 1.0
        
        #PREPARO LA RIGA DA MANDARE
        q_set = CSV[i]
        
        if username != 'ubuntu':
            q_set = sign[moveVar] * q_set                
        
        #MANDO LA RIGA
        sendFunc(q_set)  
        
        time.sleep(const * 0.004)



if __name__ == '__main__':
    if  len(sys.argv) != 4 :
        print("Incorrect number of inputs: [ 'L' or 'R' , numberOfSteps, CSVdirectoryName ]")
        exit()
    
    else: 
        directoryCSV = str(sys.argv[3])
        
        #IMPORTO I FILE: MODIFICANDO I NOMI PRESENTI IN nameDict CON LA CARTELLA DATA COME INPUT INSERENDOLI IN UN DICTIONARY INDICIZZATO DAI NOMI DI nameDict
        print('-----------------------------------')
        print('IMPORTED FILES')
        completeName = str(dirpath + '/' + directoryCSV   + '/')
        
        
        CSVdict = { nameDict[0]: np.genfromtxt( completeName +  nameDict[0] + extension , delimiter = delim)}
        for i in range(0, len(nameDict)):
                CSVdict[nameDict[i]] = np.genfromtxt(completeName + nameDict[i] + extension, delimiter = delim)
                print(completeName + nameDict[i] + extension +' ---- ' + 'Rows = ' + str(np.shape(CSVdict[nameDict[i]])[0]))
               

        #GENERO LA SEQUENZA DI INDICI APPARTENTENTI A nameDict
        moveStart = str(sys.argv[1])
        stepsNum = int(sys.argv[2])  
        sequenceDict = XcsvDICT(moveStart, stepsNum)


  	
            
        #RIFERIMENTI A ZERO
        zeroposFunc()
        
        #MANDO I CSV
        for i in range(np.shape(sequenceDict)[0]):
            if(recieved == 1):
                break
            print(str(i+1) + " / " + str(np.shape(sequenceDict)[0]) )
            moveFunc(sequenceDict[i])
		    #continue
		    # CICLO DI CORREZIONE
#			time.sleep(2.0)
#		    exit_from_cycle = False
#		    k = 0
#   	    while exit_from_cycle == False: #
#		check = 0
#		k = k+1
#		if viewFoot_DX() == True:
#			check = check + 1
#		if viewFoot_SX() == True:
#			check = check + 1
#		if check == 2 or k == 4:
#			exit_from_cycle = True
#		time.sleep(1.0)
#	    	print('##############################################################')

	
	    
