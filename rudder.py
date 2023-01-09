import RPi.GPIO as GPIO
import time
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
GPIO.setup(7, GPIO.OUT)

p = GPIO.PWM(7, 333)
p.start(1)
for i in range(0,100):
    p.ChangeDutyCycle(i)
    print(i)
    time.sleep(0.2)
input('Press return to end:')   # use raw_input for Python 2
p.stop()
GPIO.cleanup()
