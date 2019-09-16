print("start")
import RPi.GPIO as GPIO			# Import the GPIO module
import time 				    # Import the time module
GPIO.setmode(GPIO.BCM)			# Set the GPIO pin naming convention to BCM
GPIO.setup(21,GPIO.OUT)			# Set up GPIO pin 21 as an output

while True:
  print('Kick')
  GPIO.output(21,GPIO.HIGH) 	# Set GPIO pin 21 to digital high (on)
  time.sleep(2)	    			# Wait
  GPIO.output(21,GPIO.LOW)		# Set GPIO pin 21 to digital low (off)
  time.sleep(2)  

GPIO.cleanup()				# Exit the GPIO session cleanly