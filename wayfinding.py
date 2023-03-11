from clock_RV1805 import *
NOMINAL_SPEED = 0.7 
ACCEL_OFFSET_X = -0.14
ACCEL_OFFSET_Y = 0
ACCEL_OFFSET_Z = -0.32
ACCEL_GRAVITY = 9.81


def init_wayfinding(period):
    global frame_velocity_x, frame_velocity_z
    frame_velocity_x = 0 # velocity at beginning of frame
    frame_velocity_z = 0

    global collection_period 
    collection_period = period

    global total_displacement_x, total_displacement_z
    total_displacement_x = 0
    total_displacement_z = 0

    global reference_time
    reference_time = getTime_s()

def update_kinematics(IMU):
    global frame_velocity_x, frame_velocity_z
    global collection_period
    global total_displacement_x, total_displacement_z
    new_accel_x = IMU.axRaw * 9.81/4096 + ACCEL_OFFSET_X
    new_accel_z = IMU.azRaw * 9.81/4096 - ACCEL_GRAVITY + ACCEL_OFFSET_Z
    
    frame_velocity_x = frame_velocity_x + new_accel_x*collection_period # v = v_0 + at
    frame_velocity_z = frame_velocity_z + new_accel_z*collection_period # v = v_0 + at
    
    frame_displacement_x = frame_velocity_x*collection_period + 1/2*new_accel_x*collection_period**2 # dx = vt + 1/2*a*t^2
    frame_displacement_z = frame_velocity_z*collection_period + 1/2*new_accel_z*collection_period**2 # dx = vt + 1/2*a*t^2

    total_displacement_x += frame_displacement_x
    total_displacement_z += frame_displacement_z

    frame_speed = (frame_velocity_x**2 + frame_velocity_z**2)**(1/2)

    print('speed {: 3.2f}'.format(frame_speed), 'displacement z {: 3.2f}'.format(total_displacement_z))

    global reference_time
    if((reference_time + 2) < getTime_s()):
        if(frame_speed < (0.5 * NOMINAL_SPEED)):
            # Drone must be stopped
            frame_speed = 0
            frame_velocity_x = 0
            frame_velocity_z = 0
        elif(frame_speed > NOMINAL_SPEED):
            scaling_factor = NOMINAL_SPEED / frame_speed
            frame_speed *= scaling_factor
            frame_velocity_x *= scaling_factor
            frame_velocity_z *= scaling_factor
        reference_time = getTime_s()


def trapezoidal_rule(x_array, y_array):
    return 1/2*(x_array[1] - x_array[0])*(y_array[1]+y_array[0])