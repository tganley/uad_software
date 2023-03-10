import sys

sys.path.append('sensors')

from imu_icm20948 import *
from clock_RV1805 import *
from simple_pid import PID
import ms5837
import RPi.GPIO as GPIO
import time

SERVO_FREQUENCY_DEFAULT = 333 #Hz
SERVO_FREQUENCY_SET = 100 #Hz

DATA_FILE_NAME = "swim_data.txt"
IMU_COLLECTION_PERIOD = 0.4

pid_roll = PID(1, 0.1, 0.05, setpoint = 1)
pid_pitch = PID(1, 0.1, 0.05, setpoint = 2)
pid_yaw = PID(1, 0.1, 0.05, setpoint = 3)

IMU = qwiic_icm20948.QwiicIcm20948(address = 0x68)

pressure_sensor = ms5837.MS5837() # Use defaults (MS5837-30BA device on I2C bus 1)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(26, GPIO.OUT) #D7
GPIO.setup(32, GPIO.OUT) #D12
GPIO.setup(31, GPIO.OUT) #D6
GPIO.setup(36, GPIO.OUT) #D16

servoD7 = GPIO.PWM(26, SERVO_FREQUENCY_SET)
servoD12 = GPIO.PWM(32, SERVO_FREQUENCY_SET)
servoD6 = GPIO.PWM(31, SERVO_FREQUENCY_SET)
servoD16 = GPIO.PWM(36, SERVO_FREQUENCY_SET)

def system_init():
    print("This is your captain speaking. All aboard!")

    initIMU(IMU)
    pressure_sensor.init()
    initRTC()

    with open(DATA_FILE_NAME, 'a') as f:
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Timestamp", "Accel_X", "Accel_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Accel_Z", "Gyro_X", "Gyro_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t".format("Gyro_Z", "Mag_X", "Mag_Y"))
        f.write("{:>10s}\t{:>10s}\t{:>10s}\t\n".format("Mag_Z", "mbar", "Temp (C)"))



def drone_loop():

    val_roll = 12 # roll_system.update(0)
    val_pitch = 15 # pitch_system.update(0)
    val_yaw = 90 # yaw_system.update(0)
    rep = 0

    servoD7.start(1)
    servoD12.start(1)
    servoD6.start(1)
    servoD16.start(1)

    while(True):
        rep += 1


        # Collect system telemetry
        collectIMUData(IMU, IMU_COLLECTION_PERIOD)
        pressure_sensor.read(ms5837.OSR_8192)
        
        timestamp = getTime_s()
        pressure = pressure_sensor.pressure()
        temp = pressure_sensor.temperature()

        # Write to file
        with open(DATA_FILE_NAME, 'a') as file:
            file.write('{:10.2f}\t'.format(timestamp))
            writeIMUDataToFile(IMU, file)
            file.write('{:10.2f}\t'.format(pressure))
            file.write('{:10.2f}'.format(temp) + '\n')
            
        #print(str(meas) + ' mbar')
        '''
        # PID controller uses current roll value to compute new rudder control 
        control_roll = pid_roll(val_roll)
        
        # Feed PID output to rudder system and get updated roll value
        val_roll = 0 # roll_system.update(control_roll)

        # PID controller uses current pitch value to compute new rudder control 
        control_pitch = pid_pitch(val_pitch)
        
        # Feed PID output to rudder system and get updated pitch value
        val_pitch = 0 # pitch_system.update(control_pitch)

        # PID controller uses current yaw value to compute new rudder control 
        control_yaw = pid_yaw(val_yaw)
        
        # Feed PID output to rudder system and get updated yaw value
        val_yaw = 0 # yaw_system.update(control_yaw)
        
        # Check for waypoint
        '''
        servoD7.ChangeDutyCycle(5*rep % 100)
        servoD12.ChangeDutyCycle(5*rep % 100)
        servoD6.ChangeDutyCycle(5*rep % 100)
        servoD16.ChangeDutyCycle(5*rep % 100)


if __name__ == '__main__':
    try:
        system_init()
        drone_loop()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        servoD7.stop()
        servoD12.stop()
        servoD6.stop()
        servoD16.stop()
        GPIO.cleanup()
        sys.exit(0)