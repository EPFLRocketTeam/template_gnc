#!/usr/bin/env python3
from numpy.lib.type_check import real
import rospy
import rospkg

import numpy as np
import math
import serial
from scipy.interpolate import interp1d 
import time

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Sensor

import struct


current_kalman = State()

    
    
def control_callback(control):
    global current_control
    current_control = control

def simu_sensor_callback(sensor):
    # Send back sensors and control as official flight data for GNC
    sensor_pub.publish(sensor)

def kalman_callback(kalman):
    global current_kalman
    current_kalman = kalman

def fsm_callback(fsm):
    global current_fsm
    current_fsm = fsm
    


if __name__ == '__main__':

    GNC_mode = ["Flight", "HIL", "PIL", "SIL"][rospy.get_param("/simulation")]

    THROTTLING = rospy.get_param("/rocket/throttling")

    # Create global variable
    current_control = Control()

    current_fsm = FSM()
    current_fsm.state_machine = "Idle"

    

    # Init ROS
    rospy.init_node('av_interface', anonymous=True)
    
    # Subscribed topics: control, navigation state, fsm  
    rospy.Subscriber("control_pub", Control, control_callback)
    rospy.Subscriber("kalman_rocket_state", State, kalman_callback)
    rospy.Subscriber("gnc_fsm_pub", FSM, fsm_callback)

    # Published topics: sensor data, actuator feedback
    sensor_pub = rospy.Publisher('sensor_pub', Sensor, queue_size=10)
    actuator_pub = rospy.Publisher('control_measured', Control, queue_size=10)

    # If not in flight mode, we enter the ROS loop to remap sensor data from simulation to real sensor data       
    if GNC_mode == "SIL":
        measured_control = Control()
        rospy.Subscriber("simu_sensor_pub", Sensor, simu_sensor_callback)

        # Load motor thrust curve to get real thrust (for control_measured)
        rospack = rospkg.RosPack()
        thrust_curve = np.loadtxt(rospack.get_path("real_time_simulator") + "/config/thrust_curve/motor_file.txt")
        f_thrust = interp1d(thrust_curve[:,0], thrust_curve[:,1])

        # Init motor force to the one after one integration period
        current_control.force.z = rospy.get_param("/rocket/maxThrust")[2]

        rate = rospy.Rate(1.0/(2*rospy.get_param("/rocket/output_delay")))

        while not rospy.is_shutdown():
        
            # Thread sleep time defined by rate
            rate.sleep()

            if current_fsm.state_machine != "Idle":

                measured_control = current_control
                real_thrust = 0.0

                if THROTTLING:
                    real_thrust = current_control.force.z
                
                # Without throttling activated, the thrust curve is used instead
                else:
                    if current_control.force.z != 0.0 and current_fsm.time_now >= thrust_curve[0,0] and current_fsm.time_now <= thrust_curve[-1,0]:
                        real_thrust = float(f_thrust(current_fsm.time_now))

                measured_control.force.z = real_thrust
                actuator_pub.publish(measured_control)