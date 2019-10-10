import RPi.GPIO as GPIO  # Import the GPIO module
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
Motor1A = 25
Motor1B = 8
Motor1E = 7
Motor2E = 20

GPIO.setup(Motor1A, GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)

pwm = GPIO.PWM(Motor1E, 100)  # configuring Enable pin means GPIO for PWM


def dribblerOn():
    print("On")

    pwm.start(100)  # starting it with 100% dutycycle

    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor1E, GPIO.HIGH)


def dribblerOff():
    print("Off")
    GPIO.output(Motor1E, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.LOW)
    pwm.stop()  # stop PWM from GPIO output


dribblerOn()
time.sleep(10)
dribblerOff()
