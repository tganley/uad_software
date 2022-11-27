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

    IMU.begin()

def collectIMUData(collection_period):

    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.dataReady():
        IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
        print(str(getTime()))
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
        print(str(IMU.axRaw))
        with open('imu_data.txt', 'a') as f:
            f.write(str(IMU.axRaw))
            f.write('\t'.join(str(IMU.ayRaw)))
            f.write(str(IMU.azRaw))
            f.write('\n')

        time.sleep(collection_period)
    else:
        print("Waiting for data")
        time.sleep(0.5)


