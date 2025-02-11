import sys
import os
import time
import quoicouroblib as rb
import numpy as np

# Assure-toi que les modules sont correctement importés
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
import arduino_driver_v2 as arddrv
import gps_driver_v2 as gpsdrv


# Création des objets IMU et Arduino
imu = imudrv.Imu9IO()
ard = arddrv.ArduinoIO()



print("Debut")
#rb.depart()

def give_cap():
    g1_hat = np.array([0,0,0])
    mag = rb.mag()
    #print("Boussole : {}".format(mag))
    accel = rb.accel()
    #print("Acceleration : {}".format(accel))
    gyro = rb.gyro()
    #print("Gyroscope : {}".format(gyro))
    euler = rb.angles_euler_2(accel, mag,gyro,g1_hat)
    Roulis=np.degrees(euler[0])
    Tangage=np.degrees(euler[1])
    Cap=np.degrees(euler[2])
    print("Roulis : {} Tangage : {} Cap : {}".format(Roulis,Tangage,Cap))
    return ( Roulis, Tangage, Cap )
    
def mission_jour_1():
    t0=time.time()
    while (time.time()-t0)<30:
        cap=-np.pi/2
        print("suivi cap:",cap)
        rb.maintien_cap_2(rb,cap,spd_base=120)
    t0=time.time()
    while (time.time()-t0)<30:
        cap=np.pi/2
        print("suivi cap:",cap)
        rb.maintien_cap_2(rb,cap,spd_base=120)
    return

def mission_jour_2():
    t0=time.time()
    while (time.time()-t0)<30:
        mag = rb.mag()
        accel = rb.accel()
        cap=-np.pi/4
        print("suivi cap:",cap)
        rb.maintien_cap(rb,accel,mag,cap,spd_base=120)
    t0=time.time()
    while (time.time()-t0)<30:
        mag = rb.mag()
        accel = rb.accel()
        cap=np.pi/2
        print("suivi cap:",cap)
        rb.maintien_cap(rb,accel,mag,cap,spd_base=120)
    return

def detresse_cap(cap):
    while True:
        mag = rb.mag()
        accel = rb.accel()
        print("suivi cap:",cap)
        rb.maintien_cap(rb,accel,mag,cap,spd_base=120)
#mission_jour_1()
detresse_cap(np.radians(0))