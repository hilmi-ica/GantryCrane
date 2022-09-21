# Import time (simulation purpose)
import time

# Import threading 
import threading

# Import argparse (terminal command)
import argparse

# Import statistics (process data)
import statistics

# Import Numpy (math calculation)
import numpy as np

# Import independent built packages
import cv
import vacancy

# Import ROS packages
import rospy
import message_filters
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Byte
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


# Motor speed
NORMALSPEED = 40
# NORMALSPEED = 30

# PID-PD gain parameter
# Tuning Hilmi 0606 13:30
# # PSO
KP   =	8.74523890e+00
KI   =	2.38298466e-03
KD   =	1.95302109e+00
KP_S =	2.17386641e+01
KD_S =	4.58287708e+00

# # FPA
# KP   =	8.86605063e+00
# KI   =	1.56572847e-03
# KD   =	2.77376423e+00
# KP_S =	3.29976138e+01
# KD_S =	8.75100795e+00

# # SFS
# KP   =	9.03341235e+00
# KI   =	6.80362802e-05
# KD   =	1.08408526e+00
# KP_S =	7.43349592e+00
# KD_S =	9.46611818e-02

# # SA
# KP   =	8.76909010e+00
# KI   =	4.54912031e-03
# KD   =	2.32280669e+00
# KP_S =	2.56334474e+01
# KD_S =	4.28656764e+00

SAMPLING_TIME = 50
# SIMULATION_TIMEOUT = 8100
SIMULATION_TIMEOUT = 10000

# Hoisting motor related
HOISTING_MIN = 20
HOISTING_CABLE_LENGTH = 50
HOISTING_UP_SPEED = -100
HOISTING_DOWN_SPEED = 70
HOIST_MAX = 60
HOIST_MIN = 30

# PWM range
IN_MIN = 0
IN_MAX = 5
OUT_MIN = 25
# OUT_MAX = 52
# OUT_MIN = 20
OUT_MAX = 55

# Initial mode
MODE = 0

# Storing variables
class variable:
    def __init__(self,mode):
        self.mode = mode
        self.encoder = 0
        self.leftdone = False
        self.rightdone = False
        self.POSIntError = 0
        self.POSLastError = 0
        self.SWAYLastError = 0
        self.cableLength = HOISTING_CABLE_LENGTH
        self.POSsave = np.array([])
        self.SWAYsave = np.array([])
        self.SIGNALsave = np.array([])
        self.TIMESTAMPsave = np.array([])
        self.done_move = False
        self.simulationtime_counter = 0
        self.controltime_counter = 0
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
        self.depthValue = np.array([])
        self.x_set = 0
        self.hoistingdone = False
        self.stackingdone = False
        self.controldone = False
        self.vac_cont = np.array([])

var = variable(mode = MODE)

# Interpolation mapping
def val_map(x, in_min, in_max, out_min, out_max):
    return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# Time index
def millis():
    return round(time.time() * 1000)

# Step PID-PD control
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

    # np.savetxt('xpos.csv', var.POSsave, delimiter=',')
    # np.savetxt('sway.csv', var.SWAYsave, delimiter=',')

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

    var.POSsave = np.append(var.POSsave, pos)
    var.SWAYsave = np.append(var.SWAYsave, sway_disp)
    var.SIGNALsave = np.append(var.SIGNALsave, mapped)
    var.TIMESTAMPsave = np.append(var.TIMESTAMPsave,millis())

    return mapped

# Hoisting maneuver
def hoisting(y=0.64):
    # Method-1
    pwm2.publish(HOISTING_DOWN_SPEED)
    time.sleep(7)
    pwm2.publish(0)
    time.sleep(2)
    servo.publish(True)
    time.sleep(2)

    pwm2.publish(HOISTING_UP_SPEED)
    time.sleep(6)
    pwm2.publish(0)
    time.sleep(2)

    # Method-2
    # pwm2.publish(HOISTING_DOWN_SPEED)
    # h = []
    # while True:
    #     if var.ultradistValue >= y*100:
    #         pwm2.publish(0)
    #         time.sleep(2)
    #         servo.publish(True)
    #         time.sleep(2)
    #         break
        # h.append(var.ultradistValue)
        # if len(h) == 20:
        #     h = h[1:20]
        #     if statistics.mode(h) >= y*100:
        #         pwm2.publish(0)
        #         time.sleep(2)
        #         servo.publish(True)
        #         time.sleep(2)
        #         break

    # pwm2.publish(HOISTING_UP_SPEED)
    # h = []
    # while True:
    #     if var.ultradistValue <= 35:
    #         pwm2.publish(0)
    #         time.sleep(2)
    #         break
        # h.append(var.ultradistValue)
        # if len(h) == 20:
        #     h = h[1:20]
        #     if statistics.mode(h) <= 35:
        #         pwm2.publish(0)
        #         time.sleep(2)
        #         break


    # var.hoistingdone = True
    # var.controltime_counter = millis()
    return True

# Stacking maneuver
def stacking(y=0.3):
    # Method-1
    pwm2.publish(HOISTING_DOWN_SPEED)
    time.sleep(10)
    pwm2.publish(0)
    time.sleep(2)
    servo.publish(False)
    time.sleep(2)

    pwm2.publish(HOISTING_UP_SPEED)
    time.sleep(8)
    pwm2.publish(0)
    time.sleep(2)

    # Method-2
    # pwm2.publish(HOISTING_DOWN_SPEED)
    # h = []
    # while True:
    #     if var.ultradistValue >= y*100:
    #         pwm2.publish(0)
    #         time.sleep(2)
    #         servo.publish(False)
    #         time.sleep(2)
    #         break
        # h.append(var.ultradistValue)
        # if len(h) == 10:
        #     h = h[1:10]
        #     if statistics.mode(h) >= y*100:
        #         pwm2.publish(0)
        #         time.sleep(2)
        #         servo.publish(True)
        #         time.sleep(2)
        #         break

    # pwm2.publish(HOISTING_UP_SPEED)
    # h = []
    # while True:
    #     if var.ultradistValue <= 35:
    #         pwm2.publish(0)
    #         time.sleep(2)
    #         break
        # h.append(var.ultradistValue)
        # if len(h) == 10:
        #     h = h[1:10]
        #     if statistics.mode(h) <= 35:
        #         pwm2.publish(0)
        #         time.sleep(2)
        #         break

    # var.stackingdone = True
    return True

# Data acquisition
def save_data(pos,signal):
    var.POSsave = np.append(var.POSsave, pos)
    var.SWAYsave = np.append(var.SWAYsave, var.swayValue*float(0.13/(370-246)))
    var.SIGNALsave = np.append(var.SIGNALsave, signal)
    var.TIMESTAMPsave = np.append(var.TIMESTAMPsave,millis())

# Define mode value
def modeCb(m):
    var.mode = m.data
    rospy.loginfo("mode : {}".format(var.mode))

# Define left limit switch value
def lim_leftCb(limleft):
    var.lim_leftValue = limleft.data

# Define right switch value
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

# Define detected object value-1
def object_pointCb(object_point):
    # var.object_pointValue = object_point.data
    pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
    var.object_pointValue = pos - 0.493 + 0.00057 + 0.00194*object_point.data + -0.0000016*object_point.data**2 + 0.00000000181*object_point.data**3

# Define detected object value-2
# def object_pointCb(object_point):
#     if var.objectFound == False and (var.mode == 9 or var.mode == 10):

#         pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
#         var.object_pointValue = pos - 0.493 + 0.00057 + 0.00194*object_point.data + -0.0000016*object_point.data**2 + 0.00000000181*object_point.data**3
#         var.objectFound = True

#         print("Object Found")
#         print("Object Point.data, object_pointValue, pos : {} {} {}".format(object_point.data,var.object_pointValue,pos))

# Define recognized depth value
# def depthCb (depth):
#     # rospy.loginfo(depth)
#     var.depthValue = depth.data
#     # var.depthValue = np.array(map(float, var.depthValue))
#     var.depthValue = np.asarray(var.depthValue, dtype=np.float64, order='C')
#     # print(var.depthValue)
#     var.depthValue = np.reshape(var.depthValue,(1080,1920))
#     print(var.depthValue)
#     depth_metric = vacancy.convert1(var.depthValue)
#     dist_cont = vacancy.distance(depth_metric)
#     var.vac_cont = vacancy.vacancies(dist_cont)
#     var.x_set, y = vacancy.locate(var.vac_cont)
#     print(var.x_set)

# Lift-off maneuver
def lift_off():
    # Determined container locations
    x_dist = [0.235, 0.440, 0.645, 0.850, 1.055, 1.260]
    y_dist = [0.64, 0.52, 0.39, 0.25]
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if var.mode == 8:
            if var.simulation_start == False:
                var.simulationtime_counter = millis()
                var.simulation_start = True

            if var.leftdone == False or var.rightdone == False:
                pwm1.publish(-NORMALSPEED)
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                save_data(pos,-NORMALSPEED)
                if var.lim_leftValue == True or var.lim_rightValue == True:
                    pwm1.publish(0)
                    var.leftdone = True
                    var.rightdone = True
                    time.sleep(2)
                    cv.text()
                
            elif var.hoistingdone == False:
                var.hoistingdone = hoisting()
                x, y, y_real, vac, d = cv.depth()
                # cv.path(0,0,x,y_real,vac,type='frenet',unit='cm') # Not finished so not recommended to try
                var.controltime_counter = millis()

            elif var.controldone == False:
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                signal = int(PID_PDControl(x,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
                pwm1.publish(signal)
                if millis()-var.controltime_counter >= SIMULATION_TIMEOUT:
                    var.controldone = True
                    pwm1.publish(0) 
                    time.sleep(2)
            
            elif var.stackingdone == False:
                var.stackingdone = stacking(y)
                var.mode = 0
                mod.publish(var.mode)
                print("Simulation Time : {}".format(millis()-var.simulationtime_counter))
                np.savetxt('xpos.csv', var.POSsave, delimiter=',')
                np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
                np.savetxt('signal.csv', var.SIGNALsave, delimiter=',')
                np.savetxt('timestamp.csv', var.TIMESTAMPsave, delimiter=',')
                return 0

            elif millis()-var.simulationtime_counter >= 300000:
                rospy.shutdown()
        rate.sleep()

# Lift-on maneuver
def lift_on():
    # Determined container locations
    x_dist = [0.235, 0.440, 0.645, 0.850, 1.055, 1.260]
    y_dist = [0.64, 0.52, 0.39, 0.25]

    # Distance of containers from ultrasonic sensor
    y_real = [0, 0.1406, 0.2812, 0.4218]

    # Set location from terminal command
    x_set = x_dist[int(args.setpoint_x)]
    y_set = y_dist[int(args.setpoint_y)]
    y_real = y_real[int(args.setpoint_y)]

    # Initiate detection state
    detection = False
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if var.mode == 9:
            if var.simulation_start == False:
                var.simulationtime_counter = millis()
                var.simulation_start = True
                x, y, y_real1, vac, d = cv.depth()
                # cv.path(x_set,y_real,0,0,vac,type='frenet',unit='cm') # Not finished so not recommended to try
                var.controltime_counter = millis()

            # if var.vac_cont[int(args.setpoint_y)+1,int(args.setpoint_x)] == 1: # Not recommended when the depth is not accurate
            #     print("There is container above the selected setpoint")
            #     rospy.shutdown()

            if var.controldone == False:
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                signal = int(PID_PDControl(x_set,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
                pwm1.publish(signal)
                if millis()-var.controltime_counter >= SIMULATION_TIMEOUT:
                    var.controldone = True
                    pwm1.publish(0)
                    time.sleep(2)
                    cv.text()

            # elif detection == False: # Not recommended if the detection not accurate
            #     pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
            #     signal = int(PID_PDControl(var.object_pointValue,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
            #     pwm1.publish(signal)
            #     if abs(pos-var.object_pointValue) <= 0.05:
            #         detection = True
            #         pwm1.publish(0)
            #         time.sleep(2)
            #         cv.text(stack=y_set)

            elif var.hoistingdone == False:
                var.hoistingdone = hoisting(y_set)

            elif var.leftdone == False or var.rightdone == False:
                pwm1.publish(-NORMALSPEED)
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                save_data(pos,-NORMALSPEED)
                if var.lim_leftValue == True or var.lim_rightValue == True:
                    pwm1.publish(0)
                    var.leftdone = True
                    var.rightdone = True
                    time.sleep(2)
                
            elif var.stackingdone == False:
                var.stackingdone = stacking()
                var.mode = 0
                mod.publish(var.mode)
                print("Simulation Time : {}".format(millis()-var.simulationtime_counter))
                np.savetxt('xpos.csv', var.POSsave, delimiter=',')
                np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
                np.savetxt('signal.csv', var.SIGNALsave, delimiter=',')
                np.savetxt('timestamp.csv', var.TIMESTAMPsave, delimiter=',')
                return 0

            elif millis()-var.simulationtime_counter >= 300000:
                rospy.shutdown()
        rate.sleep()    

# Reshuffle maneuver
def reshuffle():
    # Determined container locations
    x_dist = [0.235, 0.440, 0.645, 0.850, 1.055, 1.260]
    # x_dist = [0.235, 0.450, 0.65, 0.855, 1.06, 1.265]
    y_dist = [0.64, 0.52, 0.39, 0.25]

    # Distance of containers from ultrasonic sensor
    y_real = [0, 0.1406, 0.2812, 0.4218]

    # set location from terminal command
    x_set = x_dist[int(args.setpoint_x)]
    y_set = y_dist[int(args.setpoint_y)]
    y_real = y_real[int(args.setpoint_y)]

    # Initial detection state
    detection = False

    # Initial control count
    control_counter = 1
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if var.mode == 10:
            if var.simulation_start == False:
                var.simulationtime_counter = millis()
                var.simulation_start = True
                x, y, y_real1, vac, d = cv.depth()
                var.controltime_counter = millis()

            if var.controldone == False and control_counter == 1:
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                signal = int(PID_PDControl(x_set,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
                pwm1.publish(signal)
                if millis()-var.controltime_counter >= SIMULATION_TIMEOUT:
                    var.controldone = True
                    pwm1.publish(0)
                    time.sleep(2)
                    cv.text(stack=y_dist[2])

            # elif detection == False: # Not recommended if the detection is not accurate
            #     pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
            #     signal = int(PID_PDControl(var.object_pointValue,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
            #     pwm1.publish(signal)
            #     if abs(pos-var.object_pointValue) <= 0.05:
            #         detection = True
            #         pwm1.publish(0)
            #         time.sleep(2)
            #         cv.text(stack=y_set)
            
            elif var.hoistingdone == False:
                var.hoistingdone = hoisting(y_set)
                var.controldone = False
                control_counter = 2
                # cv.path(x_set,y_real,x,y_real1,vac,type='frenet',unit='cm') # Not finished so not recommended to try
                var.controltime_counter = millis()

            elif var.controldone == False and control_counter == 2:
                pos = val_map(var.encoderValue, 1000, 28000, 0, 1.5)
                signal = int(PID_PDControl(x,pos,var.swayValue,KP,KI,KD,KP_S,KD_S))
                pwm1.publish(signal)
                if millis()-var.controltime_counter >= SIMULATION_TIMEOUT:
                    var.controldone = True
                    control_counter = 0
                    pwm1.publish(0)   
                    time.sleep(2)
            
            elif var.stackingdone == False:
                var.stackingdone = stacking(y)
                var.mode = 0
                mod.publish(var.mode)
                print("Simulation Time : {}".format(millis()-var.simulationtime_counter))
                np.savetxt('xpos.csv', var.POSsave, delimiter=',')
                np.savetxt('sway.csv', var.SWAYsave, delimiter=',')
                np.savetxt('signal.csv', var.SIGNALsave, delimiter=',')
                np.savetxt('timestamp.csv', var.TIMESTAMPsave, delimiter=',')
                return 0
            
            elif millis()-var.simulationtime_counter >= 300000:
                rospy.shutdown()
        rate.sleep()

# Initialize
if __name__ == "__main__":
    # Setup terminal command rule
    parser = argparse.ArgumentParser()

    parser.add_argument('-t', '--control_type',
        default='lift-off',
        help='type of action to perfome'
    )

    parser.add_argument('-x', '--setpoint_x',
        default=0,
        help='container of choice to move in x axis'
    )

    parser.add_argument('-y', '--setpoint_y',
        default=0,
        help='container of choice to move in y axis'
    )

    args = parser.parse_args()

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
    # depth = rospy.Subscriber('depth', Float32MultiArray, depthCb)
    # depth = rospy.Subscriber('depth', numpy_msg(Floats), depthCb)

    # Start operation in accordance to terminal command
    if args.control_type == 'lift-off':
        worker = threading.Thread(target=lift_off)
        worker.start()
    elif args.control_type == 'lift-on':
        worker = threading.Thread(target=lift_on)
        worker.start()
    elif args.control_type == 'reshuffle':
        worker = threading.Thread(target=reshuffle)
        worker.start()
    elif args.control_type == 'test':
        worker = threading.Thread(target=test)
        worker.start()
    else:
        print("{} type of control not found".format(args.control_type))
    
    rospy.spin()