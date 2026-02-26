from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udp:127.0.0.1:14540")
master.wait_heartbeat()

while True:
    distances = [500] * 10 + [65535] * (72 - 10)  # cm

    master.mav.obstacle_distance_send(
        time_usec=0,
        sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        distances=distances,
        increment=5,         # degrees
        min_distance=20,     # cm
        max_distance=1000,   # cm
        increment_f=0,
        angle_offset=0,
        frame=mavutil.mavlink.MAV_FRAME_BODY_FRD
    )

    time.sleep(0.1)