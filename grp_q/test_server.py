import sys
import os
import time
import quoicouroblib as rb

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import gps_driver_v2 as gpsdrv

while True:
    lat, long = rb.robot2_client_onetime("172.25.20.217")
    print("Latitude: {}, Longitude: {}".format(lat, long))
    time.sleep(2)