import socket
import threading
from analog import Analog
import sys
import time
import numpy as np
import pdb
import my_functions as mf
#UART.setup("UART5")

class MotionTrackingSocket3D(object):
    """ Threading example class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, print_flag = 0):
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        print("Motion tracking socket successfully created")                  # Start the execution
        MyRobotName = mf.read_file("my_name.txt").split()[0]
        local_config_file_name = MyRobotName + '_config.txt'
        s = mf.read_file(local_config_file_name)
        local_config = s.split(' ')
        host_name = local_config[1]
        port = int(local_config[2])
        address = (host_name,port) #1->3, 2->7
        self.sock.bind(address)      
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True  # Daemonize thread
        self.x = np.zeros(6)
        self.print_flag = print_flag
        self.stop_flag = 0
        self.thread.start() 
        

    def run(self):
        while(1):
            self.x = self.get_ground_truth_x()
            if self.print_flag == 1:
                print("\nAbsolute Angle: ", self.x, "\n")
                sys.stdout.flush()

            if self.stop_flag == 1:
                break

    def get_ground_truth_x(self):
        data, addr = self.sock.recvfrom(1024)
        ar = np.array(data.decode("utf-8").split('\\t'))
        x = ar.astype(np.float)
        return x

    def stop(self):
        self.stop_flag = 1
        self.sock.close()

    def get_average_truth_x(self):
        total_x = np.zeros(6)
        for inum in range(1,10):
            total_x = total_x + self.x
            time.sleep(0.1)
        avg_x = total_x/inum
        return avg_x
