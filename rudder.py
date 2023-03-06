import RPi.GPIO as GPIO
import time

SERVO_FREQUENCY = 333 #Hz
'''
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
# loop through 50 times, on/off for 1 second
for i in range(50):
    GPIO.output(7,True)
    time.sleep(1)
    #GPIO.output(7,False)
    #time.sleep(1)
GPIO.cleanup()

'''
GPIO.setmode(GPIO.BOARD)
GPIO.setup(26, GPIO.OUT) #D7
GPIO.setup(32, GPIO.OUT) #D12
GPIO.setup(31, GPIO.OUT) #D6
GPIO.setup(36, GPIO.OUT) #D16

servoD7 = GPIO.PWM(26, SERVO_FREQUENCY)
servoD12 = GPIO.PWM(32, SERVO_FREQUENCY)
servoD6 = GPIO.PWM(31, SERVO_FREQUENCY)
servoD16 = GPIO.PWM(36, SERVO_FREQUENCY)

servoD7.start(1)
servoD12.start(1)
servoD6.start(1)
servoD16.start(1)
for i in range(0,50):
    servoD7.ChangeDutyCycle(2*i)
    servoD12.ChangeDutyCycle(2*i)
    servoD6.ChangeDutyCycle(2*i)
    servoD16.ChangeDutyCycle(2*i)
    print(i)
    time.sleep(0.2)
for i in range(0,5):
    servoD7.ChangeDutyCycle(10*i+10)
    servoD12.ChangeDutyCycle(10*i+10)
    servoD6.ChangeDutyCycle(10*i+10)
    servoD16.ChangeDutyCycle(10*i+10)
    print(i)
    time.sleep(4)
input('Press return to end:')   # use raw_input for Python 2
servoD7.stop()
servoD12.stop()
servoD6.stop()
servoD16.stop()
GPIO.cleanup()
