# Application code for drone interfacing with Qwiic ICM20948 IMU

import time
import sys
import qwiic_icm20948
from clock_RV1805 import getTime

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
            + '\t' + '{: 06.2f}'.format(IMU.axRaw * 1/131)
            + '\t' + '{: 06.2f}'.format(IMU.ayRaw * 1/131) 
            + '\t' + '{: 06.2f}'.format(IMU.azRaw * 1/131)
            + '\t' + '{: 06.2f}'.format(IMU.gxRaw * 9.81/16348)
            + '\t' + '{: 06.2f}'.format(IMU.gyRaw * 9.81/16348) 
            + '\t' + '{: 06.2f}'.format(IMU.gzRaw * 9.81/16348)
            + '\t' + '{: 06.2f}'.format(IMU.mxRaw * 0.15)
            + '\t' + '{: 06.2f}'.format(IMU.myRaw * 0.15)
            + '\t' + '{: 06.2f}'.format(IMU.mzRaw * 0.15)
            + '\n')

        time.sleep(collection_period)
    else:
        print("Waiting for data")
        time.sleep(0.5)


