import math
import RPi.GPIO as GPIO # Import GPIO Modual



MOTORPERCENT_RADS = 7.96 #Conversion from 1 rad/s to RPM, as a percentage of maximum motor RPM
RADIUS_WHEEL = 0.016 # in meters
GPIO.setmode(GPIO.BCM) # Set GPIO Nameing Convention to BCM

Motor1a = 5
Motor1b = 6
Motor1E = 13

Motor2a = 12
Motor2b = 16
Motor2E = 20



GPIO.setup(Motor1a,GPIO.OUT)  # All pins as Outputs
GPIO.setup(Motor1b,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

GPIO.setup(Motor2a,GPIO.OUT)
GPIO.setup(Motor2b,GPIO.OUT)
GPIO.setup(Motor2E,GPIO.OUT)

# Settingup PWM

pwmA = GPIO.PWM(Motor1E, 100) # Initiates PWM signal - Phase
pwmB = GPIO.PWM(Motor2E, 100) # Initiates PWM signal - Phase


def Drive(velA, velB):

    # Convert to motor percentages
    speedA, speedB = TargetValues(velA, velB)

    DutyA = (abs(speedA)*0.2)
    DutyB = (abs(speedB)*0.2)

    if DutyA > 100:
        DutyA = 100
    if DutyB > 100:
        DutyB = 100

    

    # Below: Sets the - Velocities to flip direction of rotating wheel
    # LEFT Wheel - working
    pwmA.start(DutyA)
    if velA > 0:
        GPIO.output(Motor1a,HIGH)
        GPIO.output(Motor1b,LOW)
    else:
        GPIO.output(Motor1a,LOW)
        GPIO.output(Motor1b,LOW)
    # BACK Wheel - working
    pwmB.start(DutyB)
    if velB > 0:
        GPIO.output(Motor2a,HIGH)
        GPIO.output(Motor2b,LOW)
    else:
        GPIO.output(Motor2a,LOW)
        GPIO.output(Motor2b,LOW)


def TargetValues(velA, velB):
    # Angular Velocity
    omegL = velA / RADIUS_WHEEL
    omegR = velB / RADIUS_WHEEL

    # Power Value
    powerA = omegL * MOTORPERCENT_RADS
    powerB = omegR * MOTORPERCENT_RADS

    return powerA, powerB
    
