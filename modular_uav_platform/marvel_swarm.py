"""
Author: Chi Chu
"""

import time
import socket
import config
import cflib.crtp
import multiprocessing as mp

from marvel import Marvel
from marvel_logger import Logger
from utils import rpy2quat, quat2rpy

class Swarm():
    def __init__(self, ip, num=config.MARVEL_NUM, mode=True, control_mode='position'):
        """
            Swarm has 2 mode: Swarm and CombinedPlatform
        """
        self.marvel_num = num
        self.mode = mode   #true refers to swarm, false refers to conbinedPlatform
        self.control_mode = control_mode  #'position': x, y, z, yaw
        self._init_time()
        # self._init_logger()
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip

    def run(self, take_off_shared, switch_mode_shared, stop_shared,
            # Use for swarm
            cmd_shared, pos_shared, vel_shared, rpy_shared, agv_shared,
            pos_ref_shared, vel_ref_shared, rpy_ref_shared, agv_ref_shared,
            # Use for combined platform
            quat_base_shared, omega_base_shared, alpha_shared, beta_shared, thrust_shared, debug2_shared):
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers()
        self._connect()
        self.start_time = time.time()
        # Key Loop
        while 1:
            if stop_shared.value == 1:
                break
            self.current_time = time.time()
            if self.current_time - self.last_loop_time > self.update_time:
                self._switch_mode(switch_mode_shared)
                if self.mode:
                    if take_off_shared.value == 1:
                        # print("Take Off!")
                        quat_shared = rpy2quat(rpy_shared[0], rpy_shared[1], rpy_shared[2])
                        self._send_extpose(pos_shared, quat_shared)
                        self._send_cmd(cmd_shared)        
                else:
                    pass
                    # Send alpha, beta, thrust here
                self._record(pos_ref_shared, vel_ref_shared, rpy_ref_shared, agv_ref_shared)
                self.dt = self.current_time - self.last_loop_time
                self.last_loop_time = self.current_time
            else:
                time.sleep(0.001)
        self._stop()

    def _init_time(self):
        self.marvel_setup_time = 0.25
        self.update_time = 0.01 # 100Hz
        self.last_loop_time = time.time()
        self.current_time = self.last_loop_time
        self.dt = 0

    def _init_logger(self, path='./logs'):
        self.logger = Logger(folder_name=path)
    
    def _connect(self):
        self.marvel_swarm_handle = []
        # Scan for Crazyflies and initialize marvels
        for i in range(self.marvel_num):
            marvel = Marvel('radio://0/100/2M/E7E7E7E7E{}'.format(i), i)
            self.marvel_swarm_handle.append(marvel)
            # print("Find MARVEL num:{}".format(len(self.marvel_swarm_handle)))
            time.sleep(self.marvel_setup_time)
        print("------MARVEL Connection Initialized------")
    
    def _switch_mode(self, switch_mode_shared):
        if switch_mode_shared.value == 1:
            if self.mode == False:
                self.mode = True
                #TODO send switch command here
                self.client.connect((self.ip, 80))
                # for i in range(5):
                data = 'o'
                self.client.send(bytes(data, 'utf-8'))
                self.client.close()
        elif switch_mode_shared.value == 0:
            if self.mode == True:
                self.mode = False
                #TODO send switch command here
                self.client.connect((self.ip, 80))
                # for i in range(5):
                data = 'c'
                self.client.send(bytes(data, 'utf-8'))
                self.client.close()

    def _send_extpose(self, pos_shared, quat_shared):
        for i in range(self.marvel_num):
            self.marvel_swarm_handle[i].send_extpose(
                pos_shared[0], pos_shared[1], pos_shared[2], 
                quat_shared[1], quat_shared[2], quat_shared[3], quat_shared[0]) #qx, qy, qz, qw
    
    def _send_cmd(self, cmd_ref_shared):
        for i in range(self.marvel_num):
            if self.control_mode == 'position':
                self.marvel_swarm_handle[i].send_position_setpoint( 
                        cmd_ref_shared[0], cmd_ref_shared[1], cmd_ref_shared[2], cmd_ref_shared[3])
    
    def _record(self, pos_ref_shared, rpy_ref_shared, vel_ref_shared, agv_ref_shared):
        for i in range(self.marvel_num):
            pos_ref_shared[3*i:3*(i+1)] = self.marvel_swarm_handle[i].target_pos_and_rot[0:3]
            rpy_ref_shared[3*i:3*(i+1)] = self.marvel_swarm_handle[i].target_pos_and_rot[3:6]
            vel_ref_shared[3*i:3*(i+1)] = self.marvel_swarm_handle[i].target_velocity[0:3]
            agv_ref_shared[3*i:3*(i+1)] = self.marvel_swarm_handle[i].target_velocity[3:6]
        # self.logger.log_append(int(round((self.current_time-self.start_time) * 1000)), 
        #                     int(round((self.current_time-self.last_loop_time) * 1000)),
        #                     pos_shared[:], vel_shared[:], quat_shared[:], omega_shared[:],
        #                     pos_ref_record, rpy_ref_record, vel_ref_record, agv_ref_record)

    def _stop(self):
        for i in range(self.marvel_num):
            self.marvel_swarm_handle[i].stop_crazyflie()
        # self.logger.savelog()
        # self.logger.plot()
    


if __name__ == "__main__":
    takeoff = mp.Value('i', 0)
    stop = mp.Value('i', 0)
    
    swarm = Swarm()
    swarm.run(take_off_shared=takeoff, stop_shared=stop)