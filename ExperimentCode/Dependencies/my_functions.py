import numpy as np
from numpy import linalg
import time
import sys
import math
import cmath

global pi 
pi = np.pi

global sin
sin = np.sin

global cos
cos = np.cos

global asin
asin = np.arcsin

global acos
acos = np.arccos

global atan2
atan2 = np.arctan2

def asind(x):
    temp_theta = asin(x.real)
    return np.multiply(temp_theta,180.0/pi)

def acosd(x):
    temp_theta = acos(x.real)
    return np.multiply(temp_theta,180.0/pi)

def sind(x):
    tempx = np.multiply(x,pi/180.0)
    return sin(tempx)

def cosd(x):
    tempx = np.multiply(x,pi/180.0)
    return cos(tempx)

def tand(x):
    tempx = np.multiply(x,pi/180.0)
    return tan(tempx)

def atan2d(x,y):
    try:
        temp_theta = atan2(x.real,y.real)
        return np.multiply(temp_theta,180.0/pi)
    except:
        pdb.set_trace()

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

def wait_till(TIME_STAMP):
    tDiff = TIME_STAMP-time.time()
    while(time.time()<TIME_STAMP):
        wait_time = TIME_STAMP - time.time()
            #sys.stdout.write("Wait time: %d seconds \r" % (wait_time))
            #sys.stdout.flush()
    return tDiff

def read_file(file_name):
    f = open(file_name, "r")
    s = f.read()
    f.close()
    return s

def rotx(x):
    c = cosd(x)
    s = sind(x)
    R = np.matrix([[1,0,0], [0,c,-s], [0, s, c]])
    return R

def roty(x):
    c = cosd(x)
    s = sind(x)
    R = np.matrix([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return R

def rotz(x):
    c = cosd(x)
    s = sind(x)
    R = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R

def rotation_matrix(phi,theta):
    return roty(phi)*rotz(theta)

def coords2angles(X):
    x = X[0,0]   #x
    y = X[1,0]   #y
    z = X[2,0]   #z
    
    alpha = atan2d(y, math.sqrt(pow(x,2) + pow(z,2)))
    beta = atan2d(-z,x)
    return np.matrix([beta, alpha])

def angles2coords(X):
        alpha = X[0]
        beta = X[1]
        return np.matrix([cosd(alpha)*cosd(beta), sind(alpha),-cosd(alpha)*sind(beta)])

def generate_motor_commands(theta, u):
    R_global = rotation_matrix(0,theta)
    Ru = rotation_matrix(u[1], u[2])    #note that here is a minus sign for the modelling
    R_u_global = R_global*Ru
    next_global_angles = coords2angles(R_u_global[:,0])
    current_global_angles = coords2angles(R_global[:,0])
    return next_global_angles - current_global_angles

def apply_and_generate_motor_commands(theta,u,Gimbal):
    motor_commands = generate_motor_commands(theta, u)
    Motor_command_receiver = motor_commands[0,1]
    Motor_command_base = motor_commands[0,0]
    Gimbal.ApplyMotorCommandsSyncWithResidue([Motor_command_base, Motor_command_receiver])
    commands = [Motor_command_base,Motor_command_receiver]
    return commands

def get_roll_angle(phi, theta, alpha,beta):
        a = cosd(theta)*cosd(beta)
        b = cosd(theta)*sind(alpha)*sind(beta)
        c = sind(theta)*cosd(alpha)*sind(beta)
        quad_array = [a*a + b*b,2*a*c,c*c-b*b]
        A = quad_array[0]
        B = quad_array[1]
        C = quad_array[2]
        #check from here
        r1  = asind((-(B+cmath.sqrt(B*B-4*A*C))/(2*A)).real)
        r2  = asind((-(B-cmath.sqrt(B*B-4*A*C))/(2*A)).real)

        r = [r1,r2]
        eq = b*cosd(r) - c - a*sind(r)
        index_min = np.argmin(eq)
        return (r[index_min]).real


def generate_motor_commands_old(theta, alpha,beta, u, next_alpha, next_beta):
    R_scan_global = rotation_matrix(0,theta)
    R_scan = rotation_matrix(beta, alpha)
    gamma = get_roll_angle(0,theta,alpha, beta)
    R_roll = rotx(gamma)
    R_scan_inverse = np.linalg.inv(R_scan)
    R_mean = R_scan_global*R_roll*R_scan_inverse
    tu1 = -u[1]
    tu2 = -u[2]
    Ru = rotation_matrix(tu1, tu2)    #note that here is a minus sign for the modelling
    R_scan_next = rotation_matrix(next_beta, next_alpha)
    R_u_global = R_mean*Ru

    roll = atan2d(R_u_global[1,2],R_u_global[1,1]);
    Ru_roll = rotx(roll)
    next_global_angles = coords2angles(R_u_global*Ru_roll*R_scan_next[:,0])
    current_global_angles = coords2angles(R_scan_global[:,0])
    return next_global_angles - current_global_angles

def g_simple(x,c=29):
    c = 15
    y = np.exp(-pow((x/c),2))
    y_d = -2*(x/pow(c,2))*y
    return y,y_d

def h_simple(x,c=12):
    c = 11
    y = np.exp(-pow((x/c),2))
    y_d = -2*(x/pow(c,2))*y
    return y,y_d

def get_output_and_jacobian(alpha,beta,x_hat,c=29):
    x1 = x_hat[0]
    x2 = x_hat[1]
    x3 = x_hat[2]
    f1 = angles2coords([alpha,beta])
    f2 = angles2coords([x3,x2])
    
    X = f1*np.transpose(f2)

    grad_f2 = np.matrix([[-sind(x2)*cosd(x3),-cosd(x2)*sind(x3)],[0,cosd(x3)],[-cosd(x2)*cosd(x3), sind(x2)*sind(x3)]])
    grad_X = f1*grad_f2
    g,g_d = g_simple(acosd(X),c)
   
    if(X == 1):
        C = np.matrix([g[0,0],0,0])      #by the limit of the overall function
    else:
        d_temp = x1*g_d*(-1/math.sqrt(1-pow(X,2)))*grad_X
        C = np.matrix([g[0,0],d_temp[0,0],d_temp[0,1]])
    y = x1*g
    return y,C

def get_output_and_jacobian2D(psi, x_hat, c=15):
    x1 = x_hat[0]
    x2 = x_hat[1]
    g,g_d = g_simple(x2+psi,c)
   
    if( (x2 + psi) == 0):
        C = np.matrix([g,0])      #by the limit of the overall function
    else:
        C = np.matrix([g,x1*g_d])
    y = x1*g
    return y,C

def get_output_vector_and_jacobian_matrix2D(psi,x_hat,prev_psi, prev_x_hat, c=15):
    y1,C1 = get_output_and_jacobian2D(psi,x_hat,c)
    y2,C2 = get_output_and_jacobian2D(prev_psi,x_hat,c)

    y =np.matrix([[y1],[y2]])

    C11 = C1[0,0]
    C12 = C1[0,1]
    C21 = C2[0,0]
    C22 = C2[0,1]
    C = np.matrix([[C11,C12],[C21,C22]])
    return y,C




