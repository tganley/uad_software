# Application code for drone interfacing with Qwiic ICM20948 IMU

import time
import sys
import qwiic_icm20948
from clock_RV1805 import getTime

ACCELERATION_SCALING_FACTOR = 9.81/16348
GYROSCOPE_SCALING_FACTOR = 1/131
MAGNETOMETER_SCALING_FACTOR = 0.15


print("This is your captain speaking. All aboard!")

def initIMU():

    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. ", file=sys.stderr)
        return
    
    with open('imu_data.txt', 'a') as f:
        f.write("Timestamp        Accel_X    Accel_Y    Accel_Z    Gyro_X    Gyro_Y    Gyro_Z    Mag_X    Mag_Y    Mag_Z\n\n")
        
    IMU.begin()

def collectIMUData(collection_period):

    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.dataReady():
        IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
        timestamp = getTime()

        print(\
         '{: 06d}'.format(IMU.axRaw)\
        , '\t', '{: 06d}'.format(IMU.ayRaw)\
        , '\t', '{: 06d}'.format(IMU.azRaw)\
        , '\t', '{: 06d}'.format(IMU.gxRaw)\
        , '\t', '{: 06d}'.format(IMU.gyRaw)\
        , '\t', '{: 06d}'.format(IMU.gzRaw)\
        , '\t', '{: 06d}'.format(IMU.mxRaw)\
        , '\t', '{: 06d}'.format(IMU.myRaw)\
        , '\t', '{: 06d}'.format(IMU.mzRaw)\
        )
 
        with open('imu_data.txt', 'a') as f:
            f.write('{: .2f}'.format(timestamp)
            + '\t' + '{: 06.2f}'.format(IMU.axRaw * GYROSCOPE_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.ayRaw * GYROSCOPE_SCALING_FACTOR) 
            + '\t' + '{: 06.2f}'.format(IMU.azRaw * GYROSCOPE_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.gxRaw * ACCELERATION_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.gyRaw * ACCELERATION_SCALING_FACTOR) 
            + '\t' + '{: 06.2f}'.format(IMU.gzRaw * ACCELERATION_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.mxRaw * MAGNETOMETER_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.myRaw * MAGNETOMETER_SCALING_FACTOR)
            + '\t' + '{: 06.2f}'.format(IMU.mzRaw * MAGNETOMETER_SCALING_FACTOR)
            + '\n')

        time.sleep(collection_period)
    else:
        print("Waiting for data")
        time.sleep(0.5)


