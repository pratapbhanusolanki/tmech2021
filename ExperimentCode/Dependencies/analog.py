import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import my_functions as mf

class Analog:
	def __init__(self):
		folder_path = "/home/debian/ExperimentNew/"
		MyRobotName = mf.read_file(folder_path+"my_name.txt").split()[0]
		local_config_file_name = folder_path+MyRobotName + '_config.txt'
		s = mf.read_file(local_config_file_name)
		f = open(local_config_file_name, "r")
		s = f.read()
		local_config = s.split(' ')
		self.sensorPin = local_config[4]
		self.bot_name = local_config[0]
		self.voltage = 0
		ADC.setup()

	def getIntensity(self):
		total_intensity = 0
		sensorPin = self.sensorPin
		for inum in range(1,100):
			v = ADC.read(sensorPin)#(ADC.read(sensorPin)/1024.0)*7.6
			total_intensity = total_intensity + v

		avg_intensity = total_intensity/inum
		return avg_intensity

	def getIntensity2(self):
		total_intensity = 0
		sensorPin = self.sensorPin
		for inum in range(1,1000):
			v = ADC.read(sensorPin)#(ADC.read(sensorPin)/1024.0)*7.6
			total_intensity = total_intensity + v

		avg_intensity = total_intensity/inum
		return avg_intensity

	def printIntensityContinuous(self):
		while(1):
			print("Analog Measurement: ", self.getIntensity(), " on", self.bot_name)


	def testPort(self):
		print(ADC.read("P9_33")) #0.663736283779
		print(ADC.read("P9_35")) #0.408791214228
		print(ADC.read("P9_36")) #0.626129448414
		print(ADC.read("P9_37")) #0.927960932255
		print(ADC.read("P9_38")) #0.577777802944
		print(ADC.read("P9_39")) #0.913308918476
		print(ADC.read("P9_40")) #0.827106237411
