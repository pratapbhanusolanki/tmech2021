from dynamixel_sdk import *
import time 
from my_dynamixel_old import MyDynamixel 
import pdb
import sys, glob
import my_functions as mf


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 30           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4050            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15                # Dynamixel moving status threshold
TOP_DXL_MAXIMUM_POSITION_VALUE = 2500                # Mechanical Constraint
TOP_DXL_MINIMUM_POSITION_VALUE = 1000                # Mechanical Constraint

BASE_SIGN = 1
FINE_THRESHOLD = 10

def usleep(x): 
	time.sleep(x/1000000.0)

def read_file(file_name):
    f = open(file_name, "r")
    s = f.read()
    f.close()
    return s

folder_path = "/home/debian/ExperimentNew/"
MyRobotName = mf.read_file(folder_path+"my_name.txt").split()[0]
local_config_file_name = folder_path+MyRobotName + '_config.txt'
s = read_file(local_config_file_name)
local_config = s.split(' ')
BASE_BIAS = int((180-float(local_config[3]))*4096/360)
TOP_SIGN = int(local_config[7])
TOP_BIAS = 2033 + TOP_SIGN*int(float(local_config[9])*4096/360)

#########################
### MyMotorSystem Class ###
#########################

class MotorSystem:
	def __init__(self, device_name = DEVICENAME):    #1,2
		dev  = "/dev/ttyUSB*"
		scan = glob.glob(dev)
		device_name = scan[0]
		self.device_name  = device_name
		self.baud_rate = 1000000
		self.portHandler = PortHandler(self.device_name)
		self.packetHandler = PacketHandler(PROTOCOL_VERSION)  #Following the protocol 2.0
		self.base_motor = MyDynamixel(DXL1_ID, "Base Motor")
		self.top_motor = MyDynamixel(DXL2_ID, "Top Motor")
		self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
		self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
		self.active_residue=0.0
		self.past_residue=0.0
		self.Activate()
		

	def Activate(self):
		self.OpenPort()
		self.ChangeBaudRate()
		self.base_motor.EnableTorque(self.packetHandler, self.portHandler)
		self.base_motor.readPresentPosition(self.packetHandler, self.portHandler)
		self.top_motor.EnableTorque(self.packetHandler, self.portHandler)
		self.top_motor.readPresentPosition(self.packetHandler, self.portHandler)
		self.AddMotorToSync(self.base_motor)
		self.AddMotorToSync(self.top_motor)

	def Deactivate(self):
		self.base_motor.DisableTorque(self.packetHandler, self.portHandler)
		self.top_motor.DisableTorque(self.packetHandler, self.portHandler)
		self.groupSyncRead.clearParam()
		self.ClosePort()

	def OpenPort(self):
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			raise ValueError()

	def ClosePort(self):
		self.portHandler.clearPort()
		self.portHandler.closePort()

	# Set port baudrate
	def ChangeBaudRate(self):
		if self.portHandler.setBaudRate(self.baud_rate):
			print("Succeeded to change the baudrate")
		else:
			print("Failed to change the baudrate")
			print("Press any key to terminate...")
			getch()
			raise ValueError()

	def ApplyMotorCommandsSequential(self, angles):
		self.base_motor.rotateMotor(BASE_SIGN*angles[0], self.packetHandler, self.portHandler)
		self.top_motor.rotateMotor(TOP_SIGN*angles[1], self.packetHandler, self.portHandler)

	def ApplyMotorCommandsSyncWithResidue(self, angles):
		#print(self, self.active_residue)
		angles[0] = angles[0] + self.active_residue
		#print(angles, self.active_residue)
		self.ApplyMotorCommandsSync(angles)
		self.past_residue = self.past_residue + self.active_residue 
		self.active_residue = 0
		#print("End of ApplyMotorCommandsSyncWithResidue", self.active_residue, self.past_residue)

	def ApplyMotorCommandsSync(self, angles):
		self.RunGroupSyncMove(self.base_motor,BASE_SIGN*angles[0])
		self.RunGroupSyncMove(self.top_motor,TOP_SIGN*angles[1])
		#Actually sending the command to the motors
		continue_flag = 0
		while(continue_flag == 0):
			try:
				dxl_comm_result = self.groupSyncWrite.txPacket()
				continue_flag = 1
			except:
				pass
		if dxl_comm_result != COMM_SUCCESS:
			print("%s in ApplyMotorCommandsSync" % self.packetHandler.getTxRxResult(dxl_comm_result))
		# Clear syncwrite parameter storage
		self.groupSyncWrite.clearParam()
		while 1:
			try:
				self.PrintSyncReadStatus()
			except:
				print("Issue in Print Sync Status")
				pass
			if self.base_motor.reached_goal and self.top_motor.reached_goal:
				break

	def WriteAbsoluteAngles(self,angles):
		base_angle = angles[0]
		top_angle = angles[1]
		base_motor_command = int((BASE_SIGN*base_angle*4096)/(360) + BASE_BIAS)
		top_motor_command = int((TOP_SIGN*top_angle*4096)/(360) + TOP_BIAS)
		self.base_motor.writeGoalPosition(base_motor_command, self.packetHandler, self.portHandler)
		self.top_motor.writeGoalPosition(top_motor_command, self.packetHandler, self.portHandler)
		self.base_motor.readPresentPosition(self.packetHandler, self.portHandler)
		self.top_motor.readPresentPosition(self.packetHandler, self.portHandler)

	def get_pitch(self):
		dxl_comm_result = self.groupSyncRead.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s  get_pitch" % self.packetHandler.getTxRxResult(dxl_comm_result))
			return False
		else:
			self.RunGroupSyncRead(self.top_motor)
			pitch = (self.top_motor.present_position - TOP_BIAS)*TOP_SIGN*360.0/4096.0
			return pitch

	def get_yaw(self):
		dxl_comm_result = self.groupSyncRead.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s  get_yaw" % self.packetHandler.getTxRxResult(dxl_comm_result))
			return False
		else:
			self.RunGroupSyncRead(self.base_motor)
			yaw = (self.base_motor.present_position - BASE_BIAS)*BASE_SIGN*360.0/4096.0
			return yaw

	def WriteAbsoluteAngles2(self,angles):
		self.RunGroupSyncReach(self.base_motor,BASE_SIGN*angles[0],BASE_BIAS)
		self.RunGroupSyncReach(self.top_motor,TOP_SIGN*angles[1],TOP_BIAS)
		#Actually sending the command to the motors
		continue_flag = 0
		while(continue_flag == 0):
			try:
				dxl_comm_result = self.groupSyncWrite.txPacket()
				continue_flag = 1
			except:
				pass
		if dxl_comm_result != COMM_SUCCESS:
			print("%s Here" % self.packetHandler.getTxRxResult(dxl_comm_result))
		# Clear syncwrite parameter storage
		self.groupSyncWrite.clearParam()
		while 1:
			try:
				self.PrintSyncReadStatus()
			except:
				print("Issue in Print Sync Status")
				pass
			#Fine Reach to goal
			base_reached_goal = abs(self.base_motor.goal_position - self.base_motor.present_position)<FINE_THRESHOLD
			top_reached_goal = abs(self.top_motor.goal_position - self.top_motor.present_position)<FINE_THRESHOLD
			if base_reached_goal and top_reached_goal:
				break
	def TakeGroundPosition(self):
		self.WriteAbsoluteAngles2([0,0])

	def AddMotorToSync(self,motor):
		dxl_addparam_result = self.groupSyncRead.addParam(motor.id)
		if dxl_addparam_result != True:
			print("%s groupSyncRead addparam failed" % motor.name)
			raise ValueError()

	def PrintSyncReadStatus(self):
		dxl_comm_result = self.groupSyncRead.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s  in PrintSyncReadStatus" % self.packetHandler.getTxRxResult(dxl_comm_result))
		self.RunGroupSyncRead(self.base_motor)
		self.RunGroupSyncRead(self.top_motor)

	

	def RunGroupSyncRead(self,motor):
		dxl_getdata_result = self.groupSyncRead.isAvailable(motor.id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("%s: groupSyncRead getdata failed" % motor.name)
			#raise ValueError()
		motor.present_position = self.groupSyncRead.getData(motor.id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
		motor.residue = motor.goal_position - motor.present_position
		if abs(motor.goal_position - motor.present_position) < DXL_MOVING_STATUS_THRESHOLD:
			motor.reached_goal = True
		else:
			motor.reached_goal = False

	def RunGroupSyncMove(self,motor,angle):
		goal_position = max(min(int((angle*4096)/(360)) + motor.present_position + motor.residue,DXL_MAXIMUM_POSITION_VALUE),DXL_MINIMUM_POSITION_VALUE)
		param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
		dxl_addparam_result = self.groupSyncWrite.addParam(motor.id, param_goal_position)
		if dxl_addparam_result != True:
			print("%s groupSyncWrite addparam failed" % motor.name)
			#raise ValueError()
		motor.goal_position = goal_position

	def RunGroupSyncReach(self,motor,angle,bias):
		goal_position = max(min(int((angle*4096)/(360)+bias),DXL_MAXIMUM_POSITION_VALUE),DXL_MINIMUM_POSITION_VALUE)
		param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
		dxl_addparam_result = self.groupSyncWrite.addParam(motor.id, param_goal_position)
		if dxl_addparam_result != True:
			print("%s groupSyncWrite addparam failed" % motor.name)
			#raise ValueError()
		motor.goal_position = goal_position



#We need to look the files clear_multi_turn.py and sync_read_write.py/bulk_read_write.py