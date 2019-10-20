import RPi.GPIO as GPIO  # Import the GPIO module
import time  # Import the time module

print("Setup")
GPIO.setmode(GPIO.BCM)
Motor1A = 5
Motor1B = 6
Motor1E = 13

Motor2A = 12
Motor2B = 16
Motor2E = 20

GPIO.setmode(GPIO.BCM)  # GPIO Numbering

GPIO.setup(Motor1A, GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)

GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)

pwm1 = GPIO.PWM(Motor1E, 100)  # configuring Enable pin means GPIO for PWM
pwm2 = GPIO.PWM(Motor2E, 100)

Running = True

while Running:  # making a loop
    pwm1.start(100)  # starting it with 80% dutycycle
    pwm2.start(100)
    print("Fast")

    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor1E, GPIO.HIGH)

    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    GPIO.output(Motor2E, GPIO.HIGH)

    time.sleep(5)

    pwm1.ChangeDutyCycle(15)  # starting it with 25% dutycycle
    pwm2.ChangeDutyCycle(15)
    print("Slow")

    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor1E, GPIO.HIGH)

    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    GPIO.output(Motor2E, GPIO.HIGH)

    time.sleep(10)

    # pwm1.ChangeDutyCycle(15) # starting it with 25% dutycycle
    # pwm2.ChangeDutyCycle(15)
    print("Left")

    GPIO.output(Motor1A, GPIO.LOW)
    GPIO.output(Motor1B, GPIO.HIGH)
    GPIO.output(Motor1E, GPIO.HIGH)

    GPIO.output(Motor2A, GPIO.HIGH)
    GPIO.output(Motor2B, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.HIGH)

    time.sleep(5)

    # pwm1.ChangeDutyCycle(15) # starting it with 25% dutycycle
    # pwm2.ChangeDutyCycle(15)
    print("Right")

    GPIO.output(Motor1A, GPIO.HIGH)
    GPIO.output(Motor1B, GPIO.LOW)
    GPIO.output(Motor1E, GPIO.HIGH)

    GPIO.output(Motor2A, GPIO.LOW)
    GPIO.output(Motor2B, GPIO.HIGH)
    GPIO.output(Motor2E, GPIO.HIGH)

    time.sleep(5)

    print("Stop")

    GPIO.output(Motor1E, GPIO.LOW)
    GPIO.output(Motor2E, GPIO.LOW)
    pwm1.stop()  # stop PWM from GPIO output
    pwm2.stop()
    GPIO.cleanup()
    Running = False
    break
