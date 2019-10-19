import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
Motor1A = 19
Motor1B = 26

GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1B,GPIO.OUT)

# pwm=GPIO.PWM(Motor1E,100) # configuring Enable pin means GPIO for PWM

def dribblerOn():
    print("On")

    pwm.start(100) # starting it with 100% dutycycle

    GPIO.output(Motor1A,GPIO.HIGH)
    GPIO.output(Motor1B,GPIO.LOW)

def dribblerOff():
    print("Off")
    GPIO.output(Motor1A,GPIO.LOW)
    GPIO.output(Motor1B,GPIO.LOW)
    pwm.stop() # stop PWM from GPIO output
