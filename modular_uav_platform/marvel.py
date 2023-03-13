"""
Author: Chi Chu
Last Update Date: 1/30/2023

This is the class of marvel, a single modular uav platform.
Marvel: Class of the single modular UAV.

"""

import time
import logging
import numpy as np
import random

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

from marvel_logger import Logger
from utils import rad2deg

logging.basicConfig(level=logging.ERROR)

class Marvel:
    def __init__(self, link_uri, index):
        """ Initialize"""

        self._cf = Crazyflie(rw_cache='./cache')

        # Add some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.console.receivedChar.add_callback(self._console_callback)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        self.index = index
        
        # Log's variables
        self.timestamp = np.zeros([1])
        self.motor = np.zeros([4])              
        self.pos_and_rot = np.zeros([6])        
        self.velocity = np.zeros([6])           
        self.target_pos_and_rot = np.zeros([6])
        self.target_velocity = np.zeros([6])
        self.gimbal_value = np.zeros([4])

        # Quad PID Params
        self.attitude_gain_name = ['roll_kp', 'roll_ki', 'roll_kd', 'pitch_kp', 'pitch_ki', 'pitch_kd', 'yaw_kp', 'yaw_ki', 'yaw_kd']
        self.attitude_gain_value = [6.0, 3.0, 0.0, 6.0, 3.0, 0.0, 6.0, 1.0, 0.35]
        # self.attitude_gain_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.rate_gain_name = ['roll_kp', 'roll_ki', 'roll_kd', 'pitch_kp', 'pitch_ki', 'pitch_kd', 'yaw_kp', 'yaw_ki', 'yaw_kd']
        self.rate_gain_value = [250.0, 500.0, 2.5, 250.0, 500.0, 2.5, 120.0, 16.7, 0.0]
        # self.rate_gain_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_gain_name = ['vxKp', 'vxKi', 'vxKd', 'vyKp', 'vyKi', 'vyKd', 'vzKp', 'vzKi', 'vzKd']
        self.velocity_gain_value = [25.0, 1.0, 0.0, 25.0, 1.0, 0.0, 3.0, 1.0, 1.5]
        # self.velocity_gain_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.position_gain_name = ['xKp', 'xKi', 'xKd', 'yKp', 'yKi', 'yKd', 'zKp', 'zKi', 'zKd']
        self.position_gain_value = [2.0, 0.0, 0.0, 2.0, 0.0, 0.0, 2.0, 0.5, 0.0]
        # self.position_gain_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Gimbal PID Params
        self.c1, self.c2 = 3, 1
        self.gimbal_gain_name = ['pgaina', 'igaina', 'dgaina', 'pgainb', 'igainb', 'dgainb', 
							    'pgainas', 'igainas', 'dgainas', 'pgainbs', 'igainbs', 'dgainbs',
							    's_tx', 's_ty', 's_tz']    
        self.gimbal_gain_value = [850, 12, 0.4, 500, 10, 0.3,
						   0.00007/self.c1, 0.00006/self.c1, 0.000003/self.c1, 0.00007/self.c2, 0.00007/self.c2, 0.000003/self.c2,
						   1, 1, 1]
        # self.gimbal_gain_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1]

    def _connected(self, link_uri):
        """Print uri and initialize a logger"""
        
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        print('Connected to %s' % link_uri)
        # self._cf.param.add_update_callback(group='pid_attitude', name=None, cb=self._param_callback)
        # self._cf.param.add_update_callback(group='pid_rate', name=None, cb=self._param_callback)
        # self._cf.param.add_update_callback(group='velCtlPid', name=None, cb=self._param_callback)
        # self._cf.param.add_update_callback(group='posCtlPid', name=None, cb=self._param_callback)
        # self._cf.param.add_update_callback(group='sparam_ppid', name=None, cb=self._param_callback)

        # # Define logconfig
        # # Attention: each logconfig object can add max to 6 variables
        self._lg_vbattery = LogConfig(name='vbattery', period_in_ms=10)
        self._lg_vbattery.add_variable('pm.vbat', 'FP16')
        self._lg_pos_and_rot = LogConfig(name='pos_and_rot', period_in_ms=10)
        self._lg_pos_and_rot.add_variable('stateEstimate.x', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.y', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.z', 'float') 
        self._lg_pos_and_rot.add_variable('stateEstimate.roll', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.pitch', 'float')
        self._lg_pos_and_rot.add_variable('stateEstimate.yaw', 'float')
        self._lg_velocity = LogConfig(name='velocity', period_in_ms=10)
        self._lg_velocity.add_variable('stateEstimate.vx', 'float')
        self._lg_velocity.add_variable('stateEstimate.vy', 'float')
        self._lg_velocity.add_variable('stateEstimate.vz', 'float')
        self._lg_velocity.add_variable('controller.r_roll', 'float')
        self._lg_velocity.add_variable('controller.r_pitch', 'float')
        self._lg_velocity.add_variable('controller.r_yaw', 'float')
        self._lg_target_pos_and_rot = LogConfig(name='target_pos_and_rot', period_in_ms=10)
        self._lg_target_pos_and_rot.add_variable('ctrltarget.x', 'float')
        self._lg_target_pos_and_rot.add_variable('ctrltarget.y', 'float')
        self._lg_target_pos_and_rot.add_variable('ctrltarget.z', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.roll', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.pitch', 'float')
        self._lg_target_pos_and_rot.add_variable('controller.yaw', 'float')
        self._lg_target_velocity = LogConfig(name='target_velocity', period_in_ms=10)
        self._lg_target_velocity.add_variable('ctrltarget.vx', 'float')
        self._lg_target_velocity.add_variable('ctrltarget.vy', 'float')
        self._lg_target_velocity.add_variable('ctrltarget.vz', 'float')
        self._lg_target_velocity.add_variable('controller.rollRate', 'float')
        self._lg_target_velocity.add_variable('controller.pitchRate', 'float')
        self._lg_target_velocity.add_variable('controller.yawRate', 'float')
        # self._lg_motor = LogConfig(name='motor', period_in_ms=10)
        # self._lg_motor.add_variable('motor.m1', 'uint32_t')
        # self._lg_motor.add_variable('motor.m2', 'uint32_t')
        # self._lg_motor.add_variable('motor.m3', 'uint32_t')
        # self._lg_motor.add_variable('motor.m4', 'uint32_t')
        # self._lg_gimbal = LogConfig(name='sctrl_ppid', period_in_ms=10)
        # self._lg_gimbal.add_variable('sctrl_ppid.t_ae', 'float')
        # self._lg_gimbal.add_variable('sctrl_ppid.t_be', 'float')
        # self._lg_gimbal.add_variable('sctrl_ppid.u_alpha', 'float')
        # self._lg_gimbal.add_variable('sctrl_ppid.u_beta', 'float')

        try:
            self._cf.log.add_config(self._lg_vbattery)
            self._cf.log.add_config(self._lg_pos_and_rot)
            self._cf.log.add_config(self._lg_velocity)
            self._cf.log.add_config(self._lg_target_pos_and_rot)
            self._cf.log.add_config(self._lg_target_velocity)
            # self._cf.log.add_config(self._lg_motor)
            # self._cf.log.add_config(self._lg_gimbal)
            # This callback will receive the data
            self._lg_vbattery.data_received_cb.add_callback(self._battery_log_data)
            self._lg_pos_and_rot.data_received_cb.add_callback(self._pos_and_rot_log_data)
            self._lg_velocity.data_received_cb.add_callback(self._velocity_log_data)
            self._lg_target_pos_and_rot.data_received_cb.add_callback(self._target_pos_and_rot_log_data)
            self._lg_target_velocity.data_received_cb.add_callback(self._target_velocity_log_data)
            # self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
            # self._lg_gimbal.data_received_cb.add_callback(self._gimbal_log_data)
            # This callback will be called on errors
            self._lg_vbattery.error_cb.add_callback(self._log_error)
            self._lg_pos_and_rot.error_cb.add_callback(self._log_error)
            self._lg_velocity.error_cb.add_callback(self._log_error)
            self._lg_target_pos_and_rot.error_cb.add_callback(self._log_error)
            self._lg_target_velocity.error_cb.add_callback(self._log_error)
            # self._lg_motor.error_cb.add_callback(self._log_error)
            # self._lg_gimbal.error_cb.add_callback(self._log_error)
            # Start the logging
            self._lg_vbattery.start()
            self._lg_pos_and_rot.start()
            self._lg_velocity.start()
            self._lg_target_pos_and_rot.start()
            self._lg_target_velocity.start()
            # self._lg_motor.start()
            # self._lg_gimbal.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
    
    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _console_callback(self, text):
        '''A callback to run when we get console text from Crazyflie'''
        # We do not add newlines to the text received, we get them from the
        # Crazyflie at appropriate places.
        print(text, end='')
        # pass
    
    def _param_callback(self, name, value):
        print('Readback: {0}={1}'.format(name, value))

    def _battery_log_data(self, timestamp, data, logconf):
        """Report battery voltage information once"""
        battery_data = round(data['pm.vbat'], 1)
        print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
        self._lg_vbattery.data_received_cb.remove_callback(self._battery_log_data)

    def _pos_and_rot_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.pos_and_rot[0] = data['stateEstimate.x']                #[m]
        self.pos_and_rot[1] = data['stateEstimate.y']
        self.pos_and_rot[2] = data['stateEstimate.z']
        self.pos_and_rot[3] = data['stateEstimate.roll']             #[deg]
        self.pos_and_rot[4] = data['stateEstimate.pitch']
        self.pos_and_rot[5] = data['stateEstimate.yaw']

    def _velocity_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.velocity[0] = data['stateEstimate.vx']                  #[m/s]
        self.velocity[1] = data['stateEstimate.vy']
        self.velocity[2] = data['stateEstimate.vz']
        self.velocity[3] = rad2deg(data['controller.r_roll'])        #[radian/s]->[deg/s]
        self.velocity[4] = rad2deg(data['controller.r_pitch'])
        self.velocity[5] = rad2deg(data['controller.r_yaw'])
    
    def _target_pos_and_rot_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.target_pos_and_rot[0] = data['ctrltarget.x']            #[m]
        self.target_pos_and_rot[1] = data['ctrltarget.y']            
        self.target_pos_and_rot[2] = data['ctrltarget.z']
        self.target_pos_and_rot[3] = data['controller.roll']         #[deg or no scale just number]
        self.target_pos_and_rot[4] = data['controller.pitch']
        self.target_pos_and_rot[5] = data['controller.yaw']
    
    def _target_velocity_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.target_velocity[0] = data['ctrltarget.vx']              #[m/s or no scale just number], body coordinate
        self.target_velocity[1] = data['ctrltarget.vy']
        self.target_velocity[2] = data['ctrltarget.vz']
        self.target_velocity[3] = data['controller.rollRate']        #[No scale, just number]
        self.target_velocity[4] = data['controller.pitchRate']
        self.target_velocity[5] = data['controller.yawRate']

    def _motor_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.motor[0] = data['motor.m1']
        self.motor[1] = data['motor.m2']
        self.motor[2] = data['motor.m3']
        self.motor[3] = data['motor.m4']
    
    def _gimbal_log_data(self, timestamp, data, logconf):
        self.timestamp = timestamp
        self.gimbal_value[0] = data['sctrl_ppid.t_ae']
        self.gimbal_value[1] = data['sctrl_ppid.t_be']
        self.gimbal_value[2] = data['sctrl_ppid.u_alpha']
        self.gimbal_value[3] = data['sctrl_ppid.u_beta']
        # print(data['sctrl_ppid.u_alpha'])
    
    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def activate_kalman_estimator(self):
        # Set estimator to kalman estimator
        self._cf.param.set_value('stabilizer.estimator', '2')
        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        self._cf.param.set_value('locSrv.extQuatStdDev', 0.06)

    def set_controller(self, controller):
        # Set estimator to kalman estimator
        if controller == True:
            self._cf.param.set_value('stabilizer.controller', '1')
        else:
            self._cf.param.set_value('stabilizer.controller', '4')

    def send_extpos(self, x, y, z):
        self._cf.extpos.send_extpos(x, y, z)

    def send_extpose(self, x, y, z, qx, qy, qz, qw):
        self._cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

    #####-----Command-----#####
    def send_position_setpoint(self, x, y, z, yaw):
        self._cf.commander.send_position_setpoint(x=x, y=y, z=z, yaw=yaw)
    
    def send_attitude_setpoint(self, roll, pitch, yawrate, thrust):
        self._cf.commander.send_attitude_setpoint(roll, pitch, yawrate, thrust)

    def send_twoD_setpoint(self, index, w, x, y, z, alpha, beta, thrust):
        self._cf.commander.send_twoD_setpoint(index, w, x, y, z, alpha, beta, thrust)
    
    def send_setpoint(self, roll, pitch, yawrate, thrust):
        self._cf.commander.send_setpoint(roll=roll, pitch=pitch, yawrate=yawrate, thrust=thrust)

    def stop_crazyflie(self):
        self._cf.commander.send_stop_setpoint()
    
    #####-----Set PID-----#####
    def set_quad_pid(self):
        self._cf.param.set_value('posCtlPid.thrustBase', 36000) #36000
        self._cf.param.set_value('posCtlPid.thrustMin', 20000) #20000
        #attitude
        for n in self.attitude_gain_name:
            ind = self.attitude_gain_name.index(n)
            self._cf.param.set_value('pid_attitude.{}'.format(n), '{}'.format(self.attitude_gain_value[ind]))
        #angular v
        for n in self.rate_gain_name:
            ind = self.rate_gain_name.index(n)
            self._cf.param.set_value('pid_rate.{}'.format(n), '{}'.format(self.rate_gain_value[ind]))
        #vel
        for n in self.velocity_gain_name:
            ind = self.velocity_gain_name.index(n)
            self._cf.param.set_value('velCtlPid.{}'.format(n), '{}'.format(self.velocity_gain_value[ind]))
        #position
        for n in self.position_gain_name:
            ind = self.position_gain_name.index(n)
            self._cf.param.set_value('posCtlPid.{}'.format(n), '{}'.format(self.position_gain_value[ind]))

    def set_gimbal_pid(self):
        for n in self.gimbal_gain_name:
            ind = self.gimbal_gain_name.index(n)
            self._cf.param.set_value('sparam_ppid.{}'.format(n), '{}'.format(self.gimbal_gain_value[ind]))

if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    link_uri = 'radio://0/100/2M/E7E7E7E7E0'
    marvel = Marvel(link_uri, 0)
    logger = Logger()
    current_time, last_loop_time, start_time = time.time(), time.time(), time.time()
    update_time = 0.01
    pos_shared, vel_shared = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    rpy_shared, agv_shared = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    while 1:
        rand_pos, rand_vel = random.random()/1000, random.random()/1000
        rand_rpy, rand_agv = random.random()/1000, random.random()/1000
        pos_shared[0] += rand_pos
        pos_shared[1] += rand_pos*2
        pos_shared[2] += rand_pos*4
        vel_shared[0] += rand_vel
        vel_shared[1] += rand_vel*2
        vel_shared[2] += rand_vel*4
        rpy_shared[0] += rand_rpy
        rpy_shared[1] += rand_rpy*2
        rpy_shared[2] += rand_rpy*4
        agv_shared[0] += rand_agv
        agv_shared[1] += rand_agv*2
        agv_shared[2] += rand_agv*4
        
        current_time = time.time()
        if current_time - last_loop_time > update_time:
            marvel.send_position_setpoint(0.1, 0.2, 0.3, 0.4)
            pos_ref_shared = marvel.target_pos_and_rot[0:3]
            rpy_ref_shared = marvel.target_pos_and_rot[3:6]
            vel_ref_shared = marvel.target_velocity[0:3]
            agv_ref_shared = marvel.target_velocity[3:6]
            # Ture time from second to milisecond
            logger.log_append(int(round((current_time-start_time) * 1000)), int(round((current_time-last_loop_time) * 1000)),
                                    pos_shared, vel_shared, rpy_shared, agv_shared,
                                    pos_ref_shared, rpy_ref_shared,
                                    vel_ref_shared, agv_ref_shared)
            last_time = current_time
        else:
            time.sleep(0.001)
        if current_time - start_time > 5:
            marvel.stop_crazyflie()
            logger.savelog()
            logger.plot()
            break
