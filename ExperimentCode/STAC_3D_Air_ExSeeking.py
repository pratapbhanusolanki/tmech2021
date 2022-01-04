#this file contains a common tracking code for both elevator and rover 
#It checks variable from file config.npy to figure out its own type
import time
from datetime import datetime  
import subprocess
import numpy as np
from numpy import linalg
from numpy.linalg import inv
import math
import cmath
import linalgfunc
import pdb
import os
import serial
import sys, glob
import random
import Adafruit_BBIO.GPIO as GPIO
import pickle

#Libraries made for convenience
from analog import Analog
from motion_tracking_socket3D import MotionTrackingSocket3D
from led import LED
from trigger_socket import TriggerSocket
from motor_system import MotorSystem
import my_functions as mf

global pi 
pi = np.pi

def initialize():
    global num_iteration
    num_iteration = 200

    global t_Iter
    t_Iter = 0.5

    fs = 1/t_Iter

    global y_all
    y_all = np.zeros(num_iteration)

    global y_hpf_all
    y_hpf_all = np.zeros(num_iteration)

    global u_all
    u_all = np.zeros((num_iteration,3))

    global p_all
    p_all = np.zeros((num_iteration,2))

    global utotal_all
    utotal_all = np.zeros((num_iteration,2))

    global motor_commands_all
    motor_commands_all = np.zeros((num_iteration,2))

    global x_ground_truth_all
    x_ground_truth_all = np.zeros((num_iteration,6))

    global time_all
    time_all = np.zeros(num_iteration)

def setup():
    global receiver
    receiver = Analog()
    global Gimbal
    Gimbal = MotorSystem()
    Gimbal.TakeGroundPosition()

    global motion_socket
    motion_socket = MotionTrackingSocket3D()

    global MyRobotName
    MyRobotName = mf.read_file("my_type.txt").split()[0]

    global initial_pitch
    global initial_yaw
    global perturbation_factor

    if MyRobotName == 'Rover':
        initial_pitch = 7
        initial_yaw = 7
        perturbation_factor = 4
        from underlying_robot import Robot
        global myBot
        myBot = Robot(motion_socket,MyRobotName,3,0.6)

    elif MyRobotName == 'Elevator':
        initial_pitch = 6
        initial_yaw = -8
        perturbation_factor = 3

    MyRobotName2 = mf.read_file("my_name.txt").split()[0]
    local_config_file_name = MyRobotName2 + '_config.txt'
    s = mf.read_file(local_config_file_name)
    local_config = s.split(' ')
    global bias_angle
    bias_angle = float(local_config[8])
    global receiver_sum_angle
    global base_sum_angle

    receiver_sum_angle = initial_pitch
    base_sum_angle = initial_yaw

    global communication_flag
    communication_flag = int(mf.read_file("communication_flag.txt"))
    if communication_flag == 0:
        global txLED
        txLED = LED()
        txLED.on()
    else:
        from receiver_handle import ReceiverHandle
        global RxRoutine
        RxRoutine = ReceiverHandle(scan[1])

        global TxRoutine
        TxRoutine = TransmissionHandle()
    yaw1 = Gimbal.get_yaw()
    x = motion_socket.x
    if bias_angle == 180:
        yaw2 = x[0]%360-180
    else:
        yaw2 = x[0]
    #pdb.set_trace()
    if abs(yaw1-yaw2)>1.0:
        motion_socket.stop()
        Gimbal.Deactivate()
        txLED.off()
        raise Exception("Sorry, the robot is not aligned, please correct the orientation: ",yaw2 )
    Gimbal.WriteAbsoluteAngles([initial_yaw,initial_pitch])
    x = motion_socket.x

    pitch = Gimbal.get_pitch()
    yaw = Gimbal.get_yaw()
    print('Reached absolute yaw at ',yaw,' degrees, and absolute pitch at ',pitch,' degrees')

    if bias_angle == 180:
        yaw = x[0]%360-180
    else:
        yaw = x[0]
    
    print('From Motion Tracking System yaw = ',yaw,' and pitch = ',x[1])


def trigger_setup():
    current_time = time.time()
    print("Current time: %f" %(current_time))

    global my_trigger
    my_trigger = TriggerSocket()
    print("Waiting for the starting trigger on ", MyRobotName)
    global t_START
    t_START, duty, tIdle= my_trigger.waitForTrigger()
    mf.wait_till(t_START)
    global toc 
    toc = time.time()
    print("Process triggered at time ",datetime.fromtimestamp(toc).strftime('%Y %m %d_%I:%M:%S.%f %p'), ' on ', MyRobotName)
    if MyRobotName == 'Rover':
        myBot.duty = duty
        myBot.idle_time = tIdle
        myBot.motion_state = True

def closing_setup():
    Gimbal.Deactivate()
    file_name = MyRobotName + '_3D_ExSeeking_data'
    txt_file_name =  file_name + '_recent_files_name.txt'
    zip_name =  file_name + datetime.fromtimestamp(toc).strftime('_%Y-%m-%d_%I:%M_%p.npz') 
    received_data_pkl_file_name = file_name + '_received_data' + datetime.fromtimestamp(toc).strftime('_%Y-%m-%d_%I:%M_%p.pkl') 
    iteration_num_pkl_file_name = file_name + '_iteration_nums'+ datetime.fromtimestamp(toc).strftime('_%Y-%m-%d_%I:%M_%p.pkl') 
    file2write = open(txt_file_name,'w')
    file2write.write(zip_name + ' ')
    if communication_flag == 0:
        txLED.off()
    else:
        RxRoutine.stop()
        TxRoutine.deactivate_transmission()
        file2write.write(received_data_pkl_file_name + ' ')
        file2write.write(iteration_num_pkl_file_name)
        iteration_nums = RxRoutine.iteration_nums
        received_data = RxRoutine.received_data
        #np.save('recent_file_name.npy',common_file_name)
        f = open(iteration_num_pkl_file_name,"wb")
        pickle.dump(iteration_nums,f)
        f.close()

        f = open(received_data_pkl_file_name,"wb")
        pickle.dump(received_data,f)
        f.close()
    file2write.close()
    np.savez(zip_name, u_all=u_all, y_hpf_all = y_hpf_all, y_all = y_all, time_all = time_all, \
    motor_commands_all=motor_commands_all, x_ground_truth_all=x_ground_truth_all,theta_all = theta)
    message = MyRobotName+" is Done!"
    my_trigger.sendFinisherFlag(message.encode())
    my_trigger.Deactivate()
    if MyRobotName == 'Rover':
        myBot.takeGroundPosition()

    motion_socket.stop()




#Variables Initialization

initialize()
setup()

y = 0
u2 = 0
u3 = 0
u = [0,u2,u3]
timer = np.zeros(num_iteration+1)
theta = np.zeros(num_iteration+1)
scan_psi = np.zeros(num_iteration+1)
scan_theta = np.zeros(num_iteration+1)
theta[0] = initial_pitch
scan_theta[0] = theta[0]
# ReceiverStepper.rotateMotor(-theta[0])
# receiver_sum_angle = receiver_sum_angle -theta[0]
interval = np.zeros(num_iteration)


fs = 1/t_Iter  # Sampling frequency
fp = fs/perturbation_factor #perturbation frequency
#omega = fp*pi*2
K = 5

phase1 = 0     #Azimuthal phase
phase2 = 90    #elevation phase

A = 3          #Amplitude of the ES 

p1 = A*mf.sind(phase1)
p2 = A*mf.sind(phase2)

u_total = [0,p1,p2]
previous_alpha_bias = 0
previous_beta_bias = 0
alpha_bias = p2
beta_bias = p1
motor_commands =mf.generate_motor_commands_old(theta[0], previous_alpha_bias,previous_beta_bias, u, alpha_bias, beta_bias)
Motor_command_receiver = motor_commands[0,0]
Motor_command_base = motor_commands[0,1]
base_sum_angle = base_sum_angle + Motor_command_base
receiver_sum_angle = receiver_sum_angle + Motor_command_receiver
motor_commands_all[0] = [Motor_command_base,Motor_command_receiver]
trigger_setup()
set_time = t_START + t_Iter
y_all[0] = receiver.getIntensity()
x_ground_truth_all[0] = motion_socket.x
tdiff_min = 1000
for i in range(1,num_iteration):
    #print 'i= %d' %(i)
    
    #u = [0,0,0]
    Gimbal.ApplyMotorCommandsSync([Motor_command_base, Motor_command_receiver])
    theta[i] = Gimbal.get_pitch()
    y = receiver.getIntensity()
    y_all[i] = y

    lim = max(1,i-20)

    y_hpf = y-np.mean(y_all[lim:i+1])
    y_hpf_all[i] = y_hpf
    u1 = -K*y_hpf*p1
    u2 = -K*y_hpf*p2
    u=[0,u1,u2]

    previous_alpha_bias = p2
    previous_beta_bias = p1
    p1 = A*mf.sind(phase1+i*2*fp*t_Iter*180)
    p2 = A*mf.sind(phase2+i*2*fp*t_Iter*180)
    alpha_bias = p2
    beta_bias = p1
    

    if i==-1:
        pdb.set_trace()
    motor_commands =mf.generate_motor_commands_old(theta[i], previous_alpha_bias,previous_beta_bias, u, alpha_bias, beta_bias)
    Motor_command_receiver = motor_commands[0,0]
    Motor_command_base = motor_commands[0,1]

    base_sum_angle = base_sum_angle + Motor_command_base
    receiver_sum_angle = receiver_sum_angle + Motor_command_receiver
    time_all[i] = set_time-t_START
    tDiff = mf.wait_till(set_time)
    if tDiff<tdiff_min:
        tdiff_min = tDiff
    #print "Iteration: %d, Scan_radius: %d, Angle %d" %(i,scan_radius,bias)
    x_ground_truth_all[i] = motion_socket.x
    motor_commands_all[i] = [Motor_command_base,Motor_command_receiver]
    set_time = set_time + t_Iter
    print("Iteration: %d / %d \r" % (i,num_iteration) )
    if bias_angle == 180:
        yaw = x_ground_truth_all[i,0]%360-180
    else:
        yaw = x_ground_truth_all[i,0]
    
    print('From Motion Tracking System yaw = ',yaw,' and pitch = ',x_ground_truth_all[i,1], ' tDiff ',tDiff)
    

print('Minimum wait was: ',tdiff_min)
closing_setup()
print('Done!')
