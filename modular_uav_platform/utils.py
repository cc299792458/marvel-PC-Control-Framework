"""
Author: Chi Chu
"""

import struct
import numpy as np

from math import sin, cos, atan2, asin

PI = 3.1415926

def bin2int(int_bin):
    return struct.unpack('>I', int_bin)[0]

def nparray2bin(a):
    return a.tobytes()

def bin2nparray(b):
    return np.frombuffer(b, dtype=float).reshape(13)

def int2bin(integer):
    return struct.pack(">I", integer)

def rpy2quat(roll, pitch, yaw):
    phi, theta, psi = roll, pitch, yaw
    w = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2)
    x = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2)
    y = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2)
    z = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2)

    return w, x, y, z

def quat2rpy(w, x, y, z):
    phi = atan2(2*(w*x+y*z), 1-2*(x**2+y**2))
    theta = asin(2*(w*y-z*x))
    psi = atan2(2*(w*z+x*y), 1-2*(y**2+z**2))
    roll, pitch, yaw = phi, theta, psi

    return roll, pitch, yaw

def rad2deg(angle):
    return angle*180/PI

if __name__ == '__main__':
    # Check in website: https://quaternions.online/
    # Euler Angles in ZYX order, radians.
    roll, pitch, yaw = -1.142, 1.142, 2.0
    w, x, y, z = rpy2quat(roll, pitch, yaw)
    print('w:{:.3}, x:{:.3}, y:{:.3}, z:{:.3}'.format(w, x, y, z))  #0.137, -0.628, -0.137, 0.753
    roll, pitch, yaw = 1.0, 1.142, 2.0
    w, x, y, z = rpy2quat(roll, pitch, yaw)
    print('w:{:.3}, x:{:.3}, y:{:.3}, z:{:.3}'.format(w, x, y, z))  #0.617, -0.181, 0.596, 0.481
    roll, pitch, yaw = 1.0, 1.142, -1.0
    w, x, y, z = rpy2quat(roll, pitch, yaw)
    print('w:{:.3}, x:{:.3}, y:{:.3}, z:{:.3}'.format(w, x, y, z))  #0.524, 0.581, 0.223, -0.581
    roll, pitch, yaw = 2.0, -1.5, 0.5
    w, x, y, z = rpy2quat(roll, pitch, yaw)
    print('w:{:.3}, x:{:.3}, y:{:.3}, z:{:.3}'.format(w, x, y, z))  #0.241, 0.688, -0.205, 0.654
    roll, pitch, yaw = 1.1, -1.2, 1.3
    w, x, y, z = rpy2quat(roll, pitch, yaw)
    print('w:{:.3}, x:{:.3}, y:{:.3}, z:{:.3}'.format(w, x, y, z))  #0.382, 0.635, -0.122, 0.661

    w, x, y, z = 0.714, 0.187, 0.374, 0.561
    roll, pitch, yaw = quat2rpy(w, x, y, z)
    print('roll:{:.3}, pitch:{:.3}, yaw:{:.3}'.format(roll, pitch, yaw)) #0.813, 0.331, 1.475
    w, x, y, z = 0.655, -0.171, 0.526, 0.514
    roll, pitch, yaw = quat2rpy(w, x, y, z)
    print('roll:{:.3}, pitch:{:.3}, yaw:{:.3}'.format(roll, pitch, yaw)) #0.685, 1.046, 1.737
    w, x, y, z = 0.655, -0.171, -0.526, 0.514
    roll, pitch, yaw = quat2rpy(w, x, y, z)
    print('roll:{:.3}, pitch:{:.3}, yaw:{:.3}'.format(roll, pitch, yaw)) #-1.102, -0.540, 1.667
    w, x, y, z = 0.655, -0.171, -0.526, -0.514
    roll, pitch, yaw = quat2rpy(w, x, y, z)
    print('roll:{:.3}, pitch:{:.3}, yaw:{:.3}'.format(roll, pitch, yaw)) #0.685, -1.046, -1.737
    w, x, y, z = -0.655, -0.171, -0.526, -0.514
    roll, pitch, yaw = quat2rpy(w, x, y, z)
    print('roll:{:.3}, pitch:{:.3}, yaw:{:.3}'.format(roll, pitch, yaw)) #1.102, 0.540, 1.667
