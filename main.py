import sys
import os
sys.path.append('sensors')

from imu_icm20948 import *
from clock_RV1805 import *
from wayfinding import *
from simple_pid import PID
import ms5837
import RPi.GPIO as GPIO
import time
import argparse

parser = argparse.ArgumentParser(prog='python main.py', description='Runs servo control and telemetry gathering for ECE495 capstone project', epilog='Bottom Text')
parser.add_argument("-t", "--trigger", action="store_true", help="Use flag if the trigger is connected")
parser.add_argument("-ps", "--pressuresensor", action="store_true", help="Use flag if the pressure sensor is connected")
parser.add_argument("-p", "--program", action="store", default = 1, help="Indicate test program 1, 2, or 3. Any other number will result in no servo control.", required = True)
args = parser.parse_args()

SERVO_FREQUENCY_SET = 333 #Hz
MAX_TIME_DURATION = 200

DATA_FILE_NAME = "swim_data.txt"
IMU_COLLECTION_PERIOD = 0.1
SERVO_BOUNDS = [10, 90] # Min and max PWM duty cycles for the servos, corresponding to 0 and 

# PID objects
pid_roll = PID(1, 0.1, 0.05, setpoint = 1, output_limits = SERVO_BOUNDS)
pid_pitch = PID(1, 0.1, 0.05, setpoint = 2, output_limits = SERVO_BOUNDS)
pid_yaw = PID(1, 0.1, 0.05, setpoint = 3, output_limits = SERVO_BOUNDS)

# Inertial measurement unit
IMU = QwiicIcm20948(address = 0x68)

# Pressure sensor
pressure_sensor = ms5837.MS5837() # Use defaults (MS5837-30BA device on I2C bus 1)

# Four rudder control motors
GPIO.setmode(GPIO.BOARD)
GPIO.setup(26, GPIO.OUT) # D7 PWM1
GPIO.setup(32, GPIO.OUT) # D12 PWM2
GPIO.setup(38, GPIO.OUT) # D20 PWM3
GPIO.setup(40, GPIO.OUT) # D21 PWM4
servo1 = GPIO.PWM(26, SERVO_FREQUENCY_SET)
servo2 = GPIO.PWM(32, SERVO_FREQUENCY_SET)
servo3 = GPIO.PWM(38, SERVO_FREQUENCY_SET)
servo4 = GPIO.PWM(40, SERVO_FREQUENCY_SET)
servo1.ChangeDutyCycle(50) # 90 degrees, parallel to motion
servo2.ChangeDutyCycle(50) # 90 degrees, parallel to motion
servo3.ChangeDutyCycle(50) # 90 degrees, parallel to motion
servo4.ChangeDutyCycle(50) # 90 degrees, parallel to motion

# One trigger (input)
TRIGGER_PIN_D16 = 36
GPIO.setup(TRIGGER_PIN_D16, GPIO.IN, pull_up_down=GPIO.PUD_UP) # D16 Trigger

# One motor enable (output)
MOTOR_ENABLE_PIN_D6 = 31
GPIO.setup(MOTOR_ENABLE_PIN_D6, GPIO.OUT, initial = GPIO.HIGH)


def system_init():
    print("This is your captain speaking. All aboard!")

    try:
        os.remove(DATA_FILE_NAME)
    except:
        print("No file to delete")

    initIMU(IMU)
    if(args.pressuresensor):
        pressure_sensor.init()
    initRTC()

    init_wayfinding(IMU_COLLECTION_PERIOD)

    with open(DATA_FILE_NAME, 'a') as f:
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Timestamp", "Accel_X", "Accel_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Accel_Z", "Gyro_X", "Gyro_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Gyro_Z", "Mag_X", "Mag_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t\n".format("Mag_Z", "mbar", "Temp (C)"))


def testscript1():
    # Check directional effect of duty cycle change
    d_cycle = input("Enter Duty Cycle\n")
    print("Selected", d_cycle, '\n')

    servo1.ChangeDutyCycle(int(d_cycle))
    servo2.ChangeDutyCycle(int(d_cycle))
    servo3.ChangeDutyCycle(int(d_cycle))
    servo4.ChangeDutyCycle(int(d_cycle))

def testscript2():
    # Sweep through PWM duty cycles

    for i in range(0,5):
        servo1.ChangeDutyCycle(10*i+10)
        servo2.ChangeDutyCycle(10*i+10)
        servo3.ChangeDutyCycle(10*i+10)
        servo4.ChangeDutyCycle(10*i+10)
        time.sleep(2)

def testscriptPID():
    # PID controller uses current pitch value to compute new rudder control 
    control_pitch = pid_pitch(val_pitch)
        
    # Feed PID output to rudder system and get updated pitch value
    val_pitch = 0 # pitch_system.update(control_pitch)

    # PID controller uses current yaw value to compute new rudder control 
    control_yaw = pid_yaw(val_yaw)
        
    # Feed PID output to rudder system and get updated yaw value
    val_yaw = 0 # yaw_system.update(control_yaw)
        
    # Check for waypoint


def drone_loop():

    val_pitch = 15 # pitch_system.update(0)
    val_yaw = 90 # yaw_system.update(0)
    rep = 0

    servo1.start(1)
    servo2.start(1)
    servo3.start(1)
    servo4.start(1)

    initial_time = getTime_s()

    if(args.program == '1'):     
        print("Running test script 1")
    elif(args.program == '2'):
        print("Running test script 2")
    elif(args.program == '3'):
        print("Running PID test script")

    while(True):
        # Collect system telemetry
        collectIMUData(IMU, IMU_COLLECTION_PERIOD)
        if (args.pressuresensor):
            pressure_sensor.read(ms5837.OSR_8192)
        
        timestamp = getTime_s()
        if(args.pressuresensor):
            pressure = pressure_sensor.pressure()
            temp = pressure_sensor.temperature()
        else:
            pressure = 900
            temp = 22

        # Write to file
        with open(DATA_FILE_NAME, 'a') as file:
            file.write('{:10.2f}\t'.format(timestamp))
            writeIMUDataToFile(IMU, file)
            file.write('{:10.2f}\t'.format(pressure))
            file.write('{:10.2f}'.format(temp) + '\n')

        if(args.program == '1'):     
            testscript1()
        elif(args.program == '2'):
            testscript2()
        elif(args.program == '3'):
            testscriptPID()

        if(args.trigger):
            # Turn off if the trigger is released
            if(GPIO.input(TRIGGER_PIN_D16) != GPIO.HIGH):
                exit()

        # Turn off if max duration exceeded
        if(initial_time < (getTime_s() - MAX_TIME_DURATION)):
            exit()


if __name__ == '__main__':
    try:
        system_init()

        # Wait until the trigger is pressed
        if(args.trigger):
            print("Waiting for trigger")
            while(GPIO.input(TRIGGER_PIN_D16) == GPIO.HIGH):
                time.sleep(0.5)
            print("Trigger pressed!")
        
        # Call main loop
        drone_loop()

    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        servo1.stop()
        servo2.stop()
        servo3.stop()
        servo4.stop()
        GPIO.cleanup()
        sys.exit(0)

