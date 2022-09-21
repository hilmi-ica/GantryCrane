# Import ROS packages
import rospy
import message_filters
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Byte

# Import time (calculate simulation time)
import time

# Import Numpy (math calculation)
import numpy as np

#SETPOINT IN M
SETPOINT = 1 # change if needed

# Max encoder value (for calibration)
ENCOCALIB = 28000

# Motor speed
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
KP   = 5.817591138435942888
KI   = 5.981472137038461138e-4
KD   = -1.863467370986635341e-1
KP_S = 1.158741366998004807e1
KD_S = 5.424602874698116750


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


SAMPLING_TIME = 50
STEADY_STATE_TIMEOUT = 500

# PWM Range (depend on deadband)
IN_MIN = 0
IN_MAX = 5
OUT_MIN = 17
OUT_MAX = 55

# Hoisting motor related
HOISTING_MIN = 20
HOISTING_CABLE_LENGTH = 50
HOISTING_UP_SPEED = -100
HOISTING_DOWN_SPEED = 100
HOIST_MAX = 60
HOIST_MIN = 30

# Initial mode operation
MODE = 0

# Classification and initial variable value
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
        self.done_move = False
        self.steadytime_counter = 0
        self.oldPOS = 0
        self.objectFound = False
        self.objectPoint = 0

var = variable(mode = MODE)

# Invoke node operation
rospy.init_node('control')

# Setup publisher
mod = rospy.Publisher('mode', Byte, queue_size=10)
pwm1 = rospy.Publisher('pwm1', Int16, queue_size=10)
pwm2 = rospy.Publisher('pwm2', Int16, queue_size=10)
servo = rospy.Publisher('servo', Bool, queue_size=10)
cCalib = rospy.Publisher('cCalib', UInt16, queue_size=10)

# Interpolation mapping
def val_map(x, in_min, in_max, out_min, out_max):
    return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# Time definitor
def millis():
    return round(time.time() * 1000)

# PID control (without sway)
def PIDControl(setpoint,pos,kp,ki,kd):
    
    print("######################")
    print("Setpoint: {}".format(setpoint))
    print("Position: {}".format(pos))

    error = setpoint - pos
    print("Error: {}".format(error))
    # print("Last Error: {}".format(var.POSLastError))
    # if error <= 5 and error >=-5:
    #    return 0
    
    var.POSIntError += error*SAMPLING_TIME/1000
    d_error = (error-var.POSLastError)/SAMPLING_TIME/1000
    # print("Acc. Error: {}".format(var.POSIntError))
    # print("D. Error: {}".format(d_error))

    if KI == 0 and KD == 0:
        signal = kp*error
    elif KD == 0:
        signal = kp*error + ki*var.POSIntError
    else:
        signal = kp*error + ki*var.POSIntError + kd*d_error
    var.POSLastError = error

    if error < 0:
        mapped = -val_map(-signal, IN_MIN, IN_MAX, 25, OUT_MAX)
    else:
        mapped = val_map(signal, IN_MIN, IN_MAX, 25, OUT_MAX)
    
    print("Signal: {}".format(signal))    
    print("Signal Mapped: {}".format(mapped))

    return mapped

# PID-PD control
def PID_PDControl(setpoint,pos,sway_disp,kp,ki,kd,kps,kds):
        
    print("######################")
    # print("Setpoint: {}".format(setpoint))
    # print("Position: {}".format(pos))
    # print("Sway disp: {}".format(sway_disp))
    sway_disp=sway_disp*float(0.13/(370-246))

    # if sway_disp <= 0.015:
    #     sway_disp = 0

    POSError = setpoint - pos
    SWAYError = sway_disp/var.cableLength

    

    var.POSsave = np.append(var.POSsave, pos) 
    var.SWAYsave = np.append(var.SWAYsave, sway_disp)

    np.savetxt('xpos.csv', var.POSsave, delimiter=',')
    np.savetxt('sway.csv', var.SWAYsave, delimiter=',')

    print("POS Error: {}".format(POSError))
    # print("POS Last Error: {}".format(var.POSLastError))
    print("SWAY Error: {}".format(SWAYError))
    # print("SWAY Last Error: {}".format(var.SWAYLastError))
    
    # if error <= 5 and error >=-5:
    #    return 0
    
    var.POSIntError += POSError*SAMPLING_TIME/1000
    POS_d_error = (POSError-var.POSLastError)/SAMPLING_TIME/1000
    SWAY_d_error = (SWAYError-var.SWAYLastError)/SAMPLING_TIME/1000
    
    # print("POS Acc. Error: {}".format(var.POSIntError))
    # print("POS d Error: {}".format(POS_d_error))
    # print("SWAY d Error: {}".format(SWAY_d_error))

    signal = kp*POSError + ki*var.POSIntError + kd*POS_d_error + kps*SWAYError + kds*SWAY_d_error
    
    var.POSLastError = POSError
    var.SWAYLastError = SWAYError

    if POSError <= 0:
        mapped = -val_map(-signal, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)
        # mapped = 0
        # var.mode = 0
        # mod.publish(var.mode)
    else:
        mapped = val_map(signal, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)
    
    print("Signal: {}".format(signal))    
    print("Signal Mapped: {}".format(mapped))

    return mapped
    
def callback1(limleft, limright, enco):
    # mode = 1 --> calibration
    # mode = 2 --> scanning container
    if var.mode == 1:
        if limleft.data == False and var.leftdone == False:
            pwm1.publish(int(-PWM1CALIBSPEED))
            print("Jalan ke kiri")
            print(var.leftdone)
        elif limleft.data == True and var.leftdone == False:
            cCalib.publish(int(1000))
            var.leftdone = True
            print("Mentok kiri")
            print(var.leftdone)
            pwm1.publish(int(0))
            # mod.publish(0)
            # var.mode = 0
            time.sleep(2)
        elif limright.data == False and var.rightdone == False:
            pwm1.publish(int(PWM1CALIBSPEED))
            print("Jalan ke kanan")
        elif limright.data == True and var.rightdone == False:
            var.encoder = enco.data
            pwm1.publish(int(0))
            mod.publish(0)
            var.mode = 0
            var.rightdone=True
            print("Mentok kanan")

    if var.mode == 2:
        if limleft.data == True or limright.data == True:
            pwm1.publish(0)
            rospy.loginfo("Shutting down subscriber!")
            rospy.shutdown()
        elif limright.data == False and limleft.data == False and var.objectFound == False:
            pwm1.publish(int(-SCANNINGSPEED))
            print("Scanning Container")
        elif var.objectFound == True:
            pos = val_map(enco.data, 1000, 28000, 0, 1.5)
            if millis()-var.oldTime>= SAMPLING_TIME:
                var.oldTime=millis()
                signal = int(PIDControl(var.objectPoint,pos,KP,KI,KD))
                pwm1.publish(signal)
            
            if var.oldPOS != pos:
                var.steadytime_counter = millis()
            
            var.oldPOS = pos

            if millis()-var.steadytime_counter >= STEADY_STATE_TIMEOUT:
                var.mode = 3
                mod.publish(var.mode)
                pwm1.publish(0)
                var.steadytime_counter = 0
                var.oldPOS = 0
                return 0

def callback2(object_point,enco):
    # triggered when object found in scanning mode
    if var.objectFound == False and var.mode == 2:
        pos = val_map(enco.data, 1000, 28000, 0, 1.5)
        var.objectPoint = pos - 0.493 + 0.00057 + 0.00194*object_point.data + -0.000016*object_point.data**2 + 0.00000000181*object_point.data**3
        var.objectFound = True
        print("Object Found")
        print("Object Point, pos : {} {}".format(var.objectPoint,pos))

def callback3(ultradist):
    # mode = 3 --> Hoisting Mode Down (before load pick)
    # mode = 4 --> Hoisting Mode Up (after load pick)
    # mode = 6 --> Hoisting Mode Down (before put the load)
    # mode = 7 --> Hoisting Mode Up (after put the load)

    if var.mode == 3:
    #     while (ultradist.data <= HOIST_MAX):
    #         pwm2.publish(HOISTING_DOWN_SPEED)
    #     time.sleep(2)
    #     servo.publish(True)
    #     var.mode = 4
    #     mod.publish(var.mode)
        pwm2.publish(HOISTING_DOWN_SPEED)
        time.sleep(2)
        pwm2.publish(0)
        time.sleep(2)
        servo.publish(True)
        time.sleep(2)
        var.mode = 4
        mod.publish(var.mode)

    elif var.mode == 4:
    #     while (ultradist.data >= HOIST_MIN):
    #         pwm2.publish(HOISTING_UP_SPEED)
    #     time.sleep(2)
    #     var.mode = 5
    #     mod.publish(var.mode)
        pwm2.publish(HOISTING_UP_SPEED)
        time.sleep(2)
        pwm2.publish(0)
        time.sleep(2)
        var.mode = 5
        mod.publish(var.mode)

    elif var.mode == 6:
        pwm2.publish(HOISTING_DOWN_SPEED)
        time.sleep(2)
        pwm2.publish(0)
        time.sleep(2)
        servo.publish(False)
        time.sleep(2)
        var.mode = 7
        mod.publish(var.mode)

    elif var.mode == 7:
        pwm2.publish(HOISTING_UP_SPEED)
        time.sleep(2)
        pwm2.publish(0)
        var.mode = 0
        mod.publish(var.mode)

def callback4(limleft,limright,enco,sway):
    # mode = 5 --> Move to setpoint
    if var.mode == 5:
        if limleft.data == True or limright.data == True:
            pwm1.publish(0)
            rospy.loginfo("Shutting down subscriber!")
            rospy.shutdown()

        if millis()-var.oldTime>= SAMPLING_TIME:
            var.oldTime=millis()
            pos = val_map(enco.data, 1000, 28000, 0, 1.5)
            signal = int(PID_PDControl(SETPOINT,pos,sway.data,KP,KI,KD,KP_S,KD_S))
            pwm1.publish(signal)
            
            
            print("steady counter : {}".format(millis()-var.steadytime_counter))
            if var.oldPOS != pos:
                var.steadytime_counter = millis()
            
            var.oldPOS = pos

            if millis()-var.steadytime_counter >= STEADY_STATE_TIMEOUT:
                var.mode = 6
                mod.publish(var.mode)
                pwm1.publish(0)
                np.savetxt('xpos.csv', var.POSsave, delimiter=',')
                np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
                return 0

# Mode changer
def modeCb(m):
    var.mode = m.data
    rospy.loginfo("mode : {}".format(var.mode))

# Initialize
if __name__ == '__main__':
    # Invoke Subscriber
    mode = rospy.Subscriber('mode', Byte, modeCb)
    limL = message_filters.Subscriber('lim_left', Bool)
    limR = message_filters.Subscriber('lim_right', Bool)
    enc = message_filters.Subscriber('encoder', UInt16)
    sway = message_filters.Subscriber('sway', Int16)
    ultradist = message_filters.Subscriber('ultradist', UInt16)
    object_point = message_filters.Subscriber('object_point',UInt16)

    # Assign each subscribed data to each callback function
    ts1 = message_filters.ApproximateTimeSynchronizer([limL,limR,enc], 10,  0.1, allow_headerless=True)
    ts1.registerCallback(callback1)

    ts2 = message_filters.ApproximateTimeSynchronizer([object_point,enc], 10,  0.1, allow_headerless=True)
    ts2.registerCallback(callback2)

    ts3 = message_filters.ApproximateTimeSynchronizer([ultradist], 10,  0.1, allow_headerless=True)
    ts3.registerCallback(callback3)

    ts4 = message_filters.ApproximateTimeSynchronizer([limL,limR,enc,sway], 10,  0.1, allow_headerless=True)
    ts4.registerCallback(callback4)

    # Start and stop operation
    try:
        rospy.loginfo("Started subscriber node...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down subscriber!")
        rospy.shutdown()