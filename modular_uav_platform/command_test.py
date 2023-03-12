import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    link_uri = 'radio://0/100/2M/E7E7E7E7E0'

    cf = Crazyflie(rw_cache='./cache')
    