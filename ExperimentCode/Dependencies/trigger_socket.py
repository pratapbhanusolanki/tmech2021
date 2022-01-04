import socket
import my_functions as mf
import pdb

class TriggerSocket(object):
    #This socket is created just to receive the trigger from the base to start the code.
    #It is killed after receiving the trigger. 

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        print("Trigger Socket successfully created")                  # Start the execution
        MyRobotName = mf.read_file("my_name.txt").split()[0]
        local_config_file_name = MyRobotName + '_config.txt'
        s = mf.read_file(local_config_file_name)
        local_config = s.split(' ')
        host_name = local_config[1]
        port = int(local_config[2])+20
        address = (host_name,port) #1->3, 2->7
        self.sock.bind(address)      

    def waitForTrigger(self):
        data, addr = self.sock.recvfrom(1024)
        print(repr(data), "received")
        data = data.decode("utf-8")
        tStart = int(data.split(" ")[-7])
        duty = float(data.split(" ")[-4])
        tIdle = float(data.split(" ")[-1])
        return tStart, duty, tIdle

    def sendFinisherFlag(self,message):
        addr = ('prabhanu-lab.dhcp.egr.msu.edu',5100)
        self.sock.sendto(message, addr)
        
    def Deactivate(self):
        self.sock.close()