import sys

sys.path.append('sensors')

from imu_icm20948 import *
from clock_RV1805 import *
from simple_pid import PID

IMU_COLLECTION_PERIOD = 0.4

pid_roll = PID(1, 0.1, 0.05, setpoint = 1)
pid_pitch = PID(1, 0.1, 0.05, setpoint = 2)
pid_yaw = PID(1, 0.1, 0.05, setpoint = 3)

def system_init():
    initIMU()


def drone_loop():

    val_roll = 12 # roll_system.update(0)
    val_pitch = 15 # pitch_system.update(0)
    val_yaw = 90 # yaw_system.update(0)

    while(True):
        # Collect system telemetry
        collectIMUData(IMU_COLLECTION_PERIOD)
        
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


if __name__ == '__main__':
    try:
        system_init()
        drone_loop()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)