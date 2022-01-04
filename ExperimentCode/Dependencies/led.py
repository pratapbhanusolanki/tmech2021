import Adafruit_BBIO.GPIO as GPIO
import time  

def usleep(x): 
	time.sleep(x/1000000.0)

class LED:
    def __init__(self):
        self.led_pin = "P9_24" #corresponds to TX of UART1
        GPIO.setup(self.led_pin, GPIO.OUT, GPIO.PUD_UP)
        GPIO.setup(self.led_pin, GPIO.LOW, GPIO.PUD_UP)

    def on(self):
        GPIO.setup(self.led_pin, GPIO.HIGH, GPIO.PUD_UP)
        print("LED is on")

    def off(self):
        GPIO.setup(self.led_pin, GPIO.LOW, GPIO.PUD_UP)
        print("LED is off")

