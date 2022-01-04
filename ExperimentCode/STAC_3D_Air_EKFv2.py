#This file contains a common EKF tracking code for both elevator and rover 
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

def initialize():
    global num_iteration
    num_iteration = 200

    global A 
    A = np.identity(3)

    global I
    I = np.identity(3) 

    global B
    B = np.matrix([[0,0],[1,0],[0,1]])

    global Q
    Q = np.matrix([[0.00001,0,0],[0,0.0005,0],[0,0,0.0005]])
    global Q_scaling
    Q_scaling = 1000000

    global R
    R = 1

    global P_f
    P_f = np.matrix([[0.100,0,0],[0,0.50,0],[0,0,0.50]])

    global P 
    P = P_f

    global scan_parameters_all
    scan_parameters_all = np.zeros((num_iteration,6))

    global x_hatf_all
    x_hatf_all = np.zeros((num_iteration,3))

    global x_hat_all
    x_hat_all = np.zeros((num_iteration,3))

    global x_I_hat_all
    x_hat_all = np.zeros((num_iteration,3))

    global y_hat_all
    y_hat_all = np.zeros(num_iteration)

    global y_all
    y_all = np.zeros(num_iteration)

    global eigP_all
    eigP_all = np.zeros(num_iteration)

    global Pf_all
    Pf_all = np.zeros((num_iteration,3,3))

    global P_all
    P_all = np.zeros((num_iteration,3,3))

    global C_all
    C_all = np.zeros((num_iteration,3))

    global K_all
    K_all = np.zeros((num_iteration,3))

    global u_all
    u_all = np.zeros((num_iteration,3))

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


    global scan_alternation_flag
    global c
    if MyRobotName == 'Rover':
        initial_pitch = 7
        initial_yaw = 7
        scan_alternation_flag = 1
        c = 15
        from underlying_robot import Robot
        global myBot
        myBot = Robot(motion_socket,MyRobotName,3,0.6)


    elif MyRobotName == 'Elevator':
        initial_pitch = 6
        initial_yaw = -8
        scan_alternation_flag = 0
        c = 15

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
        pdb.set_trace()
        raise Exception("Sorry, the robot is not aligned, please correct the orientation: ",yaw2)
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
    mf.wait_till(t_START+3)
    global toc 
    toc = time.time()
    print("Process triggered at time ",datetime.fromtimestamp(toc).strftime('%Y %m %d_%I:%M:%S.%f %p'), ' on ', MyRobotName)
    if MyRobotName == 'Rover':
        myBot.duty = duty
        myBot.idle_time = tIdle
        myBot.motion_state = True


def closing_setup():
    Gimbal.Deactivate()
    file_name = MyRobotName + '_3D_EKF_data'
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
    np.savez(zip_name, scan_parameters_all=scan_parameters_all, \
    x_hatf_all=x_hatf_all, x_hat_all=x_hat, Pf_all=Pf_all,\
    C_all=C_all, y_hat_all=y_hat_all,\
    y_all=y_all, P_all=P_all, K_all=K_all, timer=timer,interval = interval,\
    u_all=u_all, scan_psi_all=scan_psi,scan_theta_all=scan_theta, \
    motor_commands_all=motor_commands_all, x_ground_truth_all=x_ground_truth_all,theta_all = theta)
    
    message = MyRobotName+" is Done!"
    my_trigger.sendFinisherFlag(message.encode())
    my_trigger.Deactivate()
    if MyRobotName == 'Rover':
        myBot.takeGroundPosition()

    motion_socket.stop()


initialize()
setup()
x_ground_truth_all[0] = motion_socket.x
#Variables Initialization
diff_sum = 0
x_hat = np.zeros((num_iteration,3))
comm_array = np.zeros(7)
x_hat[0,:] = [0.5,0,0]
x_hat_k_f = [0.5,0,0]
x_I_hat = np.zeros((num_iteration,3))
x_I_hat[0,:] = x_hat[0,:]
x_hatf_all[0,:] = x_hat[0,:]
x_I_hat_k = x_hat_k_f
x_hat_k_p = x_hat_k_f
y_hat = 0
K = np.identity(3)
C = np.identity(3)
y = 0

u2 = 0
u3 = 0
u = [0,u2,u3]
psi = np.zeros(num_iteration+1)
timer = np.zeros(num_iteration+1)
theta = np.zeros(num_iteration+1)
scan_psi = np.zeros(num_iteration+1)
scan_theta = np.zeros(num_iteration+1)
difference = np.zeros(num_iteration+1)
angle_bias = np.zeros(num_iteration+1) 
difference[0] = 0.5
theta[0] = Gimbal.get_pitch()
scan_theta[0] = theta[0]
# ReceiverStepper.rotateMotor(-theta[0])
# receiver_sum_angle = receiver_sum_angle -theta[0]
interval = np.zeros(num_iteration)

disturbance = 1   #degree/second
T = 0.8
T_factor =  2 #assuming 2.5 seconds for the full circle
t_Iter = 0.5    #assigned time for 1 step
switch = 0

#scanning terms 
phi = 120
scan_radius = 4
radius = 4
bias = angle_bias[0]

k_factor = 360/phi
scan_counter = (360/phi)*scan_alternation_flag-1
pause_flag = 0
active_phase = 0
alpha_bias = 0
beta_bias = 0

Motor_command_receiver = 0
Motor_command_base = 0
termination_flag =1
is_moving = 0
if(is_moving == 0):
    min_radius = 2
else:
    min_radius = 4
max_radius = 6
Vmax = 0.0

trigger_setup()
x_ground_truth_all[0] = motion_socket.x
set_time = t_START + t_Iter +3
tdiff_min = 1000
for i in range(1,num_iteration):
    #print 'i= %d' %(i)
    
    #u = [0,0,0]
    Gimbal.ApplyMotorCommandsSync([Motor_command_base, Motor_command_receiver])
    y = receiver.getIntensity()
    theta[i] = Gimbal.get_pitch()

    if y>Vmax:
        Vmax = y

    x_hat_k_f = x_hat[i-1,:] + [0,u2,u3] 
    y_hat,C = mf.get_output_and_jacobian(alpha_bias,beta_bias,x_hat_k_f,c)
    #pdb.set_trace()
    if(active_phase == 1 and termination_flag == 1):    
        P_f = A*P*A + Q_scaling*Q
        #Filtering    
        K = P_f*np.transpose(C)*linalg.inv(C*P_f*np.transpose(C) + R)
        x_hat_k_p = np.array(np.mat(x_hat_k_f).T+K*(y-y_hat)).T[0]     #0 is added to make it a one dimensional array rather a 2D array

        if x_hat_k_p[0] < 0:
            x_hat_k_p[0] = 0
            
        x_I_hat_k = x_I_hat[i-1,:] + x_hat_k_p*interval[i-1]          
        P = (np.identity(3) - K*C)*P_f
        
        difference[i] = abs((y-y_hat)/y)
        min_ind = max(i-2,0)
        diff_sum = sum(difference[min_ind:i+1])/3

        if(diff_sum < 0.5):
            G = 0.98*pause_flag
            Gi = 0.2*pause_flag
        else:
            G = 0
            Gi = 0
        u2 = -G*x_hat_k_p[1] - Gi*x_I_hat_k[1] 
        u3 = -G*x_hat_k_p[2] - Gi*x_I_hat_k[2]
        

    else:
        P_f_partial = A[0,0]*P[0,0]*A[0,0] + Q_scaling*Q[0,0]
        P_f[0,0] = P_f_partial
        K = P_f_partial*(C[0,0])/(C[0,0]*P_f_partial*C[0,0] + R)
        x_hat_k_p[0] = x_hat_k_f[0]+K*(y-y_hat)
        x_I_hat_k = [0,0,0]
        x_I_hat_k[0] = x_I_hat[i-1,0] + x_hat_k_p[0]*interval[i-1]
        P[0,0] = (1 - K*C[0,0])*P_f_partial
        u2 = 0
        u3 = 0
    u = [0,u2,u3]
    #print 'normal_u2 %f,  normal_u3 %f' %(normal_u2, normal_u3)
    P_all[i,:,:] = P
    x_hatf_all[i,:] = x_hat_k_f
    scan_parameters_all[i,:] = [beta_bias,alpha_bias, scan_counter, active_phase, pause_flag, scan_radius]
    C_all[i,:] = C
    Pf_all[i,:,:] = P_f
    y_all[i] = y
    y_hat_all[i] = y_hat
    K_all[i,:] = np.transpose(K)
    x_I_hat[i,:] = x_I_hat_k
    x_hat[i,:] = x_hat_k_p
    u_all[i,:] = u
    motor_commands_all[i] = [Motor_command_base,Motor_command_receiver]
    toc = time.time()
    timer[i] = toc-t_START
    interval[i] = timer[i] - timer[i-1]
    if(i>0):
        T = sum(interval[1:i+1])/i
    comm_array[0] = i
    comm_array[1] = timer[i]
    comm_array[2] = x_hat[i,0]
    comm_array[3] = x_hat[i,1]
    comm_array[4] = x_hat[i,2]
    comm_array[5] = y
    comm_array[6] = y_hat
    #np.save(npy_name,comm_array)
    #sftp.put(npy_name,remote_path + npy_name)
    previous_alpha_bias = scan_radius*mf.sind(bias)
    previous_beta_bias = scan_radius*mf.cosd(bias)
    P_angles = P[1:3,1:3]
    V = np.linalg.eig(P_angles)[0] #Eigen vectors
    eigP_all[i] = max(V)                     #Max eigen vector
    scan_counter = scan_counter%(2*k_factor) + 1
    if(scan_counter == 1):
        pause_flag = 1
        if(y < 0.5*Vmax):
            termination_flag = 1

    if(scan_counter == k_factor+1):
        pause_flag = 0

    if(scan_counter == 2*k_factor):
        active_phase = 1

    if(scan_counter == k_factor+1):
        active_phase = 0
        if(i>20):             #After this it becomes adaptive
            min_ind = int(max(i-k_factor,0))
            e = sum(eigP_all[min_ind:i])/k_factor
            #radius = (min(20,max(min_radius, math.floor((e)/200)))+radius)/2
            radius = min(max_radius,max(min_radius, math.floor((e)/6000)))

        if((radius == 0) and (y > 7*Vmax)):
            print("Reached terminal condition!!!")
            termination_flag = 0 + is_moving   #It will only be zero when is moving is false

    scan_radius = pause_flag*radius*termination_flag 
    
    #Computing scanning parameters for the next iteration
    
    angle_bias[i+1] = (scan_counter-1)*phi
    bias = angle_bias[i+1]
    alpha_bias = scan_radius*mf.sind(bias)
    beta_bias = scan_radius*mf.cosd(bias)

    motor_commands =mf.generate_motor_commands_old(theta[i], previous_alpha_bias,previous_beta_bias, u, alpha_bias, beta_bias)
    Motor_command_base = motor_commands[0,0] 
    Motor_command_receiver = motor_commands[0,1]
    

    base_sum_angle = base_sum_angle + Motor_command_base
    receiver_sum_angle = receiver_sum_angle + Motor_command_receiver
    #theta[i+1] = receiver_sum_angle

    time_all[i] = set_time-t_START
    tDiff= mf.wait_till(set_time)
    if tDiff<tdiff_min:
        tdiff_min = tDiff
    #print "Iteration: %d, Scan_radius: %d, Angle %d" %(i,scan_radius,bias)
    x_ground_truth_all[i] = motion_socket.x
    set_time = set_time + t_Iter
    # sys.stdout.write("Iteration: %d / %d \r" % (i,num_iteration) )
    # #sys.stdout.write("Measurements: %f / %f \r" % (y,Vmax) )
    # sys.stdout.flush()
    print("Iteration: %d / %d \r" % (i,num_iteration) )
    if bias_angle == 180:
        yaw = x_ground_truth_all[i,0]%360-180
    else:
        yaw = x_ground_truth_all[i,0]
    
    print('From Motion Tracking System yaw = ',yaw,' and pitch = ',x_ground_truth_all[i,1], ' tDiff ',tDiff)

print('Minimum wait was: ',tdiff_min)
closing_setup()
print('Done!')
