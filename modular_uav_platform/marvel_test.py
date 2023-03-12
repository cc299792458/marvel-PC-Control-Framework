import time

import cflib
from marvel import Marvel
from marvel_logger import Logger

def send_twoD_setpoint_test(marvel):
    print('twoD test')
    index = 0
    w, x, y, z  = 1.0, 0.0, 0.0, 0.0
    alpha, beta, thrust = 0.0, 0.0, 0.3
    current_time = time.time()
    start_time = current_time
    while 1:
        if current_time - start_time < 3:
            alpha = 0
            beta = 0
        elif current_time - start_time > 9:
            break
        # elif current_time - start_time < 6:
        #     alpha = -0.2
        #     # beta = 0.2
        #     pass
        # elif current_time - start_time < 9:
        #     alpha = -0.2
        #     # beta = 0.4
        #     pass
        # elif current_time - start_time > 12:
        #     break
        # print("send")
        marvel.send_twoD_setpoint(index, w, x, y, z, alpha, beta, thrust)
        current_time = time.time()    
    marvel.stop_crazyflie()
    print('Stop')

def send_position_setpoint_test(marvel):
    print('position test')
    x, y, z, yaw = 0.0, 0.0, 0.2, 0.0
    current_time = time.time()
    start_time = current_time
    while 1:
        if current_time - start_time > 3:
            break
        marvel.send_position_setpoint(x, y, z, yaw)
        current_time = time.time()    
    marvel.stop_crazyflie()
    print('Stop')

def send_attitude_setpoint_test(marvel, logger):
    print("attitude test")
    roll_d, pitch_d, yawrate_d, thrust_d = 0.0, 0.0, 0.0, 20000
    current_time = time.time()
    start_time = current_time
    last_time = start_time
    while 1:
        if current_time - start_time > 20:
            break
        elif current_time - start_time < 5:
            roll_d = 10.0
        # elif current_time - start_time < 10:
        #     pitch_d = 0.0
        # elif current_time - start_time < 15:
            # pitch_d = 0.0
        # elif current_time - start_time < 20:
        #     pitch_d = 0.0
        marvel.send_attitude_setpoint(roll_d, pitch_d, yawrate_d, thrust_d)
        if current_time - last_time > 0.01:
            rpy_s = marvel.pos_and_rot[3:6]
            agv_s = marvel.velocity[3:6]
            rpy_t = marvel.target_pos_and_rot[3:6]
            agv_t = marvel.target_velocity[3:6]
            logger.log_append(timestamp=int(round((current_time-start_time) * 1000)), dt=int(round((current_time-last_time) * 1000)),
                                                  rpy=rpy_s, agv=agv_s, rpy_ref=rpy_t, agv_ref=agv_t)
            last_time = current_time
        current_time = time.time()    
    marvel.stop_crazyflie()
    print('Stop')
    logger.plot(type='attitude')

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    link_uri = 'radio://0/100/2M/E7E7E7E7E0'
    # link_uri = 'radio://0/80/2M/E7E7E7E7E0'
    
    marvel = Marvel(link_uri, 0)
    logger = Logger(1)
    
    time.sleep(2)
    # marvel.set_quad_pid()
    # marvel.set_gimbal_pid()
    # marvel.set_controller(True)
    # send_position_setpoint_test(marvel)
    # marvel.set_controller(False)
    # send_twoD_setpoint_test(marvel)

    send_attitude_setpoint_test(marvel, logger)
