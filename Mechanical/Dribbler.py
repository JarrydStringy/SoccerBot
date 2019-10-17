import RPi.GPIO as GPIO
import time

# GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
Motor1A = 19
Motor1B = 26

GPIO.setup(Motor1A, GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1B, GPIO.OUT)

# pwm=GPIO.PWM(Motor1E,100) # configuring Enable pin means GPIO for PWM

print("On")

# pwm.start(100) # starting it with 100% dutycycle

GPIO.output(Motor1A, GPIO.HIGH)
GPIO.output(Motor1B, GPIO.LOW)
# GPIO.output(Motor1E,GPIO.HIGH)
time.sleep(20)


GPIO.output(Motor1A, GPIO.LOW)
GPIO.output(Motor1B, GPIO.LOW)
# pwm.stop() # stop PWM from GPIO output
