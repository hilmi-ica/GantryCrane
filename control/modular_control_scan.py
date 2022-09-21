# Import ROS packages
import rospy
import message_filters
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Byte

# Import time 
import time

# Import Numpy
import numpy as np

# Import threading
import threading



#SETPOINT IN M
SETPOINT = 1

# Max value of encoder
ENCOCALIB = 28000

# Trolley motor speed
PWM1CALIBSPEED = 40
SCANNINGSPEED = 30

# PID-PD gain parameter
#============================================
#Gain settling time 4 detik
#FPA
# KP = 8.69758
# KI = 0.00290
# KD = 1.54447

# KP_S = 16.94624
# KD_S = 3.94190

# #SFS
# KP = 8.70872
# KI = 0.00594
# KD = 1.12517

# KP_S = 13.25603
# KD_S = 2.76868

#PSO
# KP = 8.69106
# KI = 0.00404
# KD = 2.27953

# KP_S = 24.45196
# KD_S = 5.50633

#============================================
#Gain settling time 6 detik
# FPA
# KP   = 5.817591138435942888
# KI   = 5.981472137038461138e-4
# KD   = -1.863467370986635341e-1
# KP_S = 1.158741366998004807e1
# KD_S = 5.424602874698116750


#PSO
# KP   = 5.814796499068507707e+00
# KI   = 7.899828721136151026e-04
# KD   = -1.659987246407810535e-01
# KP_S = 1.180063420855373835e+01
# KD_S = 5.545363948789428399e+00

#SFS
# KP   = 5.952205339205777435e+00
# KI   = 8.918540229135206660e-04
# KD   = -4.387971825014427840e-01
# KP_S = 9.616662934108392236e+00
# KD_S = 1.306250309938971288e+00



#######################
#GAIN BARU

# PSO
# KP   = 5.796375095157274338e+00
# KI   = 3.037471786110971378e-03
# KD   = -1.248736743273289457e-01
# KP_S = 1.201433580223289965e+01
# KD_S = 5.852099422141537666e+00

# SFS
# KP   = 5.749999783162103029e+00
# KI   = 4.143929291662943118e-03
# KD   = -2.574644004552782417e-01
# KP_S = 8.929714718268364493e+00
# KD_S = 5.326623189545730774e+00

# FPA
KP   = 5.812718901858813680e+00
KI   = 1.444641129244449542e-03
KD   = -1.821822576226065638e-01
KP_S = 1.155234452685554380e+01
KD_S = 5.458135132860924443e+00

# KP = 5.801966262919881601e+00
# KI = 1.270764533040685991e-15
# KD = -1.628946691076045949e+00



SAMPLING_TIME = 50
# SIMULATION_TIMEOUT = 7000
SIMULATION_TIMEOUT = 20000

# PWM range
IN_MIN = 0
IN_MAX = 5
OUT_MIN = 20
OUT_MAX = 60

# Hoisting motor related
HOISTING_MIN = 20
HOISTING_CABLE_LENGTH = 50
HOISTING_UP_SPEED = -100
HOISTING_DOWN_SPEED = 100
HOIST_MAX = 60
HOIST_MIN = 30

# Initial mode
MODE = 0

# Storing variables and their values
class variable:
    def __init__(self,mode):
        self.mode = mode
        self.encoder = 0
        self.leftdone = False
        self.rightdone = False
        self.POSIntError = 0
        self.POSLastError = 0
        self.SWAYLastError = 0
        self.oldTime = 0
        self.cableLength = HOISTING_CABLE_LENGTH
        self.POSsave = np.array([])
        self.SWAYsave = np.array([])
        self.SIGNALsave = np.array([])
        self.TIMESTAMPsave = np.array([])
        self.done_move = False
        self.simulationtime_counter = 0
        self.oldPOS = 0
        self.objectFound = False
        self.objectPoint = 0
        self.simulation_start = False
        self.encoderValue = 0
        self.swayValue = 0
        self.lim_leftValue = 0
        self.lim_rightValue = 0
        self.ultradistValue = 0
        self.object_pointValue = 0

var = variable(mode = MODE)

# rospy.init_node('control')

# Interpolation mapping
def val_map(x, in_min, in_max, out_min, out_max):
    return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# Time definitor
def millis():
    return round(time.time() * 1000)

# PID control (without sway)
# def PIDControl(setpoint,pos,kp,ki,kd):
    
#     print("######################")
#     print("Setpoint: {}".format(setpoint))
#     print("Position: {}".format(pos))

#     error = setpoint - pos
#     print("Error: {}".format(error))
#     # print("Last Error: {}".format(var.POSLastError))
#     # if error <= 5 and error >=-5:
#     #    return 0
    
#     var.POSIntError += error*SAMPLING_TIME/1000
#     d_error = (error-var.POSLastError)/SAMPLING_TIME/1000
#     # print("Acc. Error: {}".format(var.POSIntError))
#     # print("D. Error: {}".format(d_error))

#     if KI == 0 and KD == 0:
#         signal = kp*error
#     elif KD == 0:
#         signal = kp*error + ki*var.POSIntError
#     else:
#         signal = kp*error + ki*var.POSIntError + kd*d_error
#     var.POSLastError = error

#     if error < 0:
#         mapped = -val_map(-signal, IN_MIN, IN_MAX, 25, OUT_MAX)
#     else:
#         mapped = val_map(signal, IN_MIN, IN_MAX, 25, OUT_MAX)
    
#     print("Signal: {}".format(signal))    
#     print("Signal Mapped: {}".format(mapped))

#     return mapped

# PID-PD control
def PID_PDControl(setpoint,pos,kp,ki,kd):

    print("######################")
    # print("Setpoint: {}".format(setpoint))
    print("Position: {}".format(pos))
    # print("Sway disp: {}".format(sway_disp))
    # sway_disp=sway_disp*float(0.13/(370-246))

    # if sway_disp <= 0.015:
    #     sway_disp = 0

    POSError = setpoint - pos
    # SWAYError = sway_disp/var.cableLength

    

    

    # np.savetxt('xpos.csv', var.POSsave, delimiter=',')
    # np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
    
    print("POS Error: {}".format(POSError))
    # print("POS Last Error: {}".format(var.POSLastError))
    # print("SWAY Error: {}".format(SWAYError))
    # print("SWAY Last Error: {}".format(var.SWAYLastError))
    
    # if error <= 5 and error >=-5:
    #    return 0
    
    var.POSIntError += POSError*SAMPLING_TIME/1000
    POS_d_error = (POSError-var.POSLastError)/SAMPLING_TIME/1000
    # SWAY_d_error = (SWAYError-var.SWAYLastError)/SAMPLING_TIME/1000
    
    # print("POS Acc. Error: {}".format(var.POSIntError))
    # print("POS d Error: {}".format(POS_d_error))
    # print("SWAY d Error: {}".format(SWAY_d_error))

    signal = kp*POSError + ki*var.POSIntError + kd*POS_d_error # + kps*SWAYError + kds*SWAY_d_error
    
    var.POSLastError = POSError
    # var.SWAYLastError = SWAYError

    if POSError <= 0:
        mapped = -val_map(-signal, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)
        # mapped = 0
        # var.mode = 0
        # mod.publish(var.mode)
    else:
        mapped = val_map(signal, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)
    
    print("Signal: {}".format(signal))    
    print("Signal Mapped: {}".format(mapped))

    var.POSsave = np.append(var.POSsave, pos)
    # var.SWAYsave = np.append(var.SWAYsave, sway_disp)=
    var.SIGNALsave = np.append(var.SIGNALsave, mapped)
    var.TIMESTAMPsave = np.append(var.TIMESTAMPsave,millis())

    return mapped

# Define mode value
def modeCb(m):
    var.mode = m.data
    rospy.loginfo("mode : {}".format(var.mode))

# Define left limit switch value
def lim_leftCb(limleft):
    var.lim_leftValue = limleft.data

# Define right limit switch value
def lim_rightCb(limright):
    var.lim_rightValue = limright.data

# Define encoder value
def encoderCb(enc):
    var.encoderValue = enc.data

# Define sway value
def swayCb(sway):
    var.swayValue = sway.data

# Define ultrasonic sensor value
def ultradistCb(ultradist):
    var.ultradistValue = ultradist.data

# Define object detection value
def object_pointCb(object_point):
    if var.objectFound == False and var.mode == 2:

        pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
        var.object_pointValue = pos - 0.493 + 0.00057 + 0.00194*object_point.data + -0.0000016*object_point.data**2 + 0.00000000181*object_point.data**3
        var.objectFound = True

        print("Object Found")
        print("Object Point.data, object_pointValue, pos : {} {} {}".format(object_point.data,var.object_pointValue,pos))

        
# Step control with object detection as setpoint control
def publisher_thread():
    rate = rospy.Rate(20) # ROS Rate at 20Hz
    while not rospy.is_shutdown():
        # mode = 2 --> Move to container detected
        if var.mode == 2:
            if var.simulation_start == False:
                var.simulationtime_counter = millis()
                var.simulation_start = True

            if var.lim_leftValue == True or var.lim_rightValue == True:
                pwm1.publish(0)
                rospy.loginfo("Shutting down subscriber!")
                rospy.shutdown()

            elif var.lim_rightValue == False and var.lim_leftValue == False and var.objectFound == False:
                pwm1.publish(int(SCANNINGSPEED))
                # print("Scanning Container")
            
            elif var.objectFound == True:
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)

                var.oldTime=millis()
                signal = int(PID_PDControl(var.object_pointValue,pos,KP,KI,KD))
                pwm1.publish(signal)
                rospy.loginfo("PUBLISH SIGNAL")
            
                if millis()-var.simulationtime_counter >= SIMULATION_TIMEOUT:
                    var.mode = 0
                    mod.publish(var.mode)
                    pwm1.publish(0)

                    np.savetxt('xpos.csv', var.POSsave, delimiter=',')
                    np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
                    np.savetxt('signal.csv', var.SIGNALsave, delimiter=',')
                    np.savetxt('timestamp.csv', var.TIMESTAMPsave, delimiter=',')

                    return 0
            
                print("Simulation Time : {}".format(millis()-var.simulationtime_counter))
        rate.sleep()

# Initialize
if __name__ == '__main__':
    # Invoke node
    rospy.init_node("control")

    # Invoke publisher
    mod = rospy.Publisher('mode', Byte, queue_size=10)
    pwm1 = rospy.Publisher('pwm1', Int16, queue_size=10)
    pwm2 = rospy.Publisher('pwm2', Int16, queue_size=10)
    servo = rospy.Publisher('servo', Bool, queue_size=10)
    cCalib = rospy.Publisher('cCalib', UInt16, queue_size=10)

    # Invoke subscriber
    mode = rospy.Subscriber('mode', Byte, modeCb)
    limL = rospy.Subscriber('lim_left', Bool, lim_leftCb)
    limR = rospy.Subscriber('lim_right', Bool, lim_rightCb)
    enc = rospy.Subscriber('encoder', UInt16, encoderCb)
    sway = rospy.Subscriber('sway', Int16, swayCb)
    ultradist = rospy.Subscriber('ultradist', UInt16, ultradistCb)
    object_point = rospy.Subscriber('object_point',UInt16, object_pointCb)

    # Start operation by threading
    worker = threading.Thread(target=publisher_thread)
    worker.start()
    
    rospy.spin()