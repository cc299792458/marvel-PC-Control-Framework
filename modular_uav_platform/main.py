"""
Author: Chi Chu
Last Update Date: 2/16/2023

This is the main script of the modular uav platform swarm.
Class Swarm: The class of marvel swarms.
Class Master: Main class, consist of keyboard, logger and swarm
Function main:  Obviously, the main function :).

User inputs:
	refer to gimbal_platform_keyboard.py for keyboard control
	Press ESC for emergency stop

"""

import time
import logging
import multiprocessing as mp
import cflib.crtp
import config  

# from vicon import Vicon
from marvel import Marvel
from marvel_swarm import Swarm
from marvel_keyboard import KeyboardInput
from marvel_logger import Logger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Master():
    def __init__(self):
        self.start_time = 0
        self.update_time = 0.005    # 200Hz

        # Reference variables, get from keyboard or single marvel
        self.pos_ref_shared, self.rpy_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.vel_ref_shared, self.agv_ref_shared = mp.Array('f',3), mp.Array('f',3)
        self.cmd_ref_shared = mp.Array('f',4)
        # Ground truth varaibles, get from vicon
        self.pos_shared, self.vel_shared = mp.Array('f',3), mp.Array('f',3)
        self.quat_shared, self.omega_shared = mp.Array('f',4), mp.Array('f',3)
        self.quat_shared[:] = [1, 0, 0, 0]
        # These command value get from keyboard
        self.take_off_shared = mp.Value('i', 0)
        self.stop_shared = mp.Value('i', 0)
        # Initialize keyboard 
        self.keyboard = KeyboardInput(control_mode='position')
        self.logger = Logger(folder_name='./logs')
        #In this framework, there is no high-level controller now
        #self.controller = Controller()
        #self.p_control = mp.Process()
        self._init_swarm()
        # self.vicon = Vicon()
        time.sleep(0.25)
    
    def run(self):
        self.start_time = time.time()
        self.last_loop_time = self.start_time
        print("Start time:", self.start_time)
        while 1:
            # print(self.pos_shared[:])
            if self.keyboard.stop == 1:
                print("Program has been safely stopped.")
                break
            current_time = time.time()
            if current_time - self.last_loop_time > self.update_time:
                # Share vicon value
                # self.pos_shared[:] = self.vicon.position
                # self.vel_shared[:] = self.vicon.velocity
                # self.quat_shared[:] = self.vicon.rotation
                # self.omega_shared[:] = self.vicon.rotation_rate
                # Share keyboard values
                self.keyboard.command.update()
                # Share command value
                for i in range(config.MARVEL_NUM):
                    if self.swarm.mode == 0:
                        self.cmd_ref_shared[:] = self.keyboard.cmd
                    elif self.swarm.mode == 1:
                        self.pos_ref_shared[:] = self.keyboard.pos
                        self.rpy_ref_shared[:] = self.keyboard.rpy
                        self.vel_ref_shared[:] = self.keyboard.vel
                        self.agv_ref_shared[:] = self.keyboard.agv
                self.take_off_shared.value = self.keyboard.take_off
                self.stop_shared.value = self.keyboard.stop
                # Log values
                if self.swarm.mode == 1:
                    self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
                                            self.pos_shared[:], self.vel_shared[:], self.quat_shared[:], self.omega_shared[:],
                                            self.pos_ref_shared[:], self.rpy_ref_shared[:],
                                            self.vel_ref_shared[:], self.agv_ref_shared[:])
                self.last_loop_time = current_time
            else:
                time.sleep(0.0001)
        self._stop()

    def _init_swarm(self):
        self.swarm = Swarm()
        print("------First Step Initialization Completed------")
        self.p_swarm = mp.Process(target=self.swarm.run, args=(self.pos_ref_shared, self.rpy_ref_shared, 
                                                                self.vel_ref_shared, self.agv_ref_shared,
                                                                self.cmd_ref_shared,
                                                                self.pos_shared, self.vel_shared,
                                                                self.quat_shared, self.omega_shared,
                                                                self.take_off_shared, self.stop_shared))

    def _stop(self):
        self.keyboard.command.quit()
        self.keyboard.command.destroy()
        # self.logger.savelog()
        # self.logger.plot()

        # if self.swarm.mode == 0:
        #     self.swarm.logger.savelog()
        #     self.logger.plot()
        # elif self.swarm.mode == 1:
        #     self.logger.savelog()
        #     self.logger.plot()
        time.sleep(0.1)

def main():
    master = Master()
    master.p_swarm.start()
    sleepTime = 6
    for i in range(sleepTime):
        print("Wait for Launch......:{}".format(sleepTime-i))
        time.sleep(1)
    # print("Verify handle num:{}".format(len(master.swarm.marvel_swarm_handle)))    
    master.run()

    master.p_swarm.join()

if __name__ == '__main__':
    main()