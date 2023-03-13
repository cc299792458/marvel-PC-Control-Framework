"""
Author: Chi Chu
Date: 2/22/2023

This script maintains the log class of the modular uav swarm.
Class Logger: 

"""

import os
import csv
import time
import config
import numpy as np
import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('Agg')

class Logger():
    def __init__(self, num=config.MARVEL_NUM, folder_name='./logs'):
        self.marvel_num = num
        self.folder_name = folder_name
        self.log_memory_title = ["timestamp", "dt"]
        for i in range(self.marvel_num):
            self.log_memory_title.append("pos_x_{}".format(i))
            self.log_memory_title.append("pos_y_{}".format(i))
            self.log_memory_title.append("pos_z_{}".format(i))
            self.log_memory_title.append("vel_x_{}".format(i))
            self.log_memory_title.append("vel_y_{}".format(i))
            self.log_memory_title.append("vel_z_{}".format(i))
            self.log_memory_title.append("roll_{}".format(i))
            self.log_memory_title.append("pitch_{}".format(i))
            self.log_memory_title.append("yaw_{}".format(i))
            self.log_memory_title.append("agv_x_{}".format(i))
            self.log_memory_title.append("agv_y_{}".format(i))
            self.log_memory_title.append("agv_z_{}".format(i))
            self.log_memory_title.append("x_ref_{}".format(i))
            self.log_memory_title.append("y_ref_{}".format(i))
            self.log_memory_title.append("z_ref_{}".format(i))
            self.log_memory_title.append("roll_ref_{}".format(i))
            self.log_memory_title.append("pitch_ref_{}".format(i))
            self.log_memory_title.append("yaw_ref_{}".format(i))
            self.log_memory_title.append("x_vel_ref_{}".format(i))
            self.log_memory_title.append("y_vel_ref_{}".format(i))
            self.log_memory_title.append("z_vel_ref_{}".format(i))
            self.log_memory_title.append("agv_x_ref_{}".format(i))
            self.log_memory_title.append("agv_y_ref_{}".format(i))
            self.log_memory_title.append("agv_z_ref_{}".format(i))
        self.log_memory = [self.log_memory_title]
        self.log_base_memory = [["timestamp", "dt",
                                "pos_x", "pos_y", "pos_z",
                                "vel_x", "vel_y", "vel_z",
                                "roll", "pitch", "yaw",
                                "agv_x", "agv_y", "agv_z",
                                "x_ref", "y_ref", "z_ref",
                                "roll_ref", "pitch_ref", "yaw_ref",
                                "x_vel_ref", "y_vel_ref", "z_vel_ref",
                                "agv_x_ref", "agv_y_ref", "agv_z_ref"]]

    def log_append(self, timestamp, dt, pos=[0.0,0.0,0.0], vel=[0.0,0.0,0.0], rpy=[0.0,0.0,0.0], agv=[0.0,0.0,0.0], 
                                    pos_ref=[0.0,0.0,0.0], rpy_ref=[0.0,0.0,0.0], vel_ref=[0.0,0.0,0.0], agv_ref=[0.0,0.0,0.0]):
        #TODO:
        #for i in range(self.marvel_num):
        self.log_memory.append([timestamp, dt,
                                pos[0], pos[1], pos[2],
                                vel[0], vel[1], vel[2],
                                rpy[0], rpy[1], rpy[2],
                                agv[0], agv[1], agv[2],
                                pos_ref[0], pos_ref[1], pos_ref[2], 
								rpy_ref[0], rpy_ref[1], rpy_ref[2], 
								vel_ref[0], vel_ref[1], vel_ref[2],
								agv_ref[0], agv_ref[1], agv_ref[2]])

    def log_base_append(self, timestamp, dt, 
                        pos, vel, rpy, agv, 
                        pos_ref, rpy_ref, vel_ref, agv_ref):
        self.log_base_memory.append([timestamp, dt,
                                pos[0], pos[1], pos[2],
                                vel[0], vel[1], vel[2],
                                rpy[0], rpy[1], rpy[2],
                                agv[0], agv[1], agv[2],
                                pos_ref[0], pos_ref[1], pos_ref[2], 
								rpy_ref[0], rpy_ref[1], rpy_ref[2], 
								vel_ref[0], vel_ref[1], vel_ref[2],
								agv_ref[0], agv_ref[1], agv_ref[2]])

    def openCSVMatrix(self, filename = 'filename'):
        rows = []
        with open(filename) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                newrow = []
                for cell in row:
                    newcell = self.conv(cell)
                    newrow.append(newcell)
                rows.append(newrow)
        M = np.mat(rows)
        return M

    def savelog(self):
        try:
			# Create target Directory
            os.mkdir(self.folder_name)
            print("Directory \"" + self.folder_name +  "\" Created ") 
        except:
			# print("Directory " + self.folder_name +  " already exists")
            pass
		
        with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S") + '.txt', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.log_memory)
            print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")
        with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S") + '.txt', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.log_base_memory)
            print("CSV file: " + "log_base_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")
    
    def plot(self, type=None):
        n = len(self.log_memory)
        log_memory_array = np.asarray(self.log_memory[1:n])
        timestamp = log_memory_array[:, 0] - log_memory_array[0, 0]
        i = 2
        pos_x = log_memory_array[:, i]; i+=1
        pos_y = log_memory_array[:, i]; i+=1
        pos_z = log_memory_array[:, i]; i+=1
        vel_x = log_memory_array[:, i]; i+=1
        vel_y = log_memory_array[:, i]; i+=1
        vel_z = log_memory_array[:, i]; i+=1
        roll = log_memory_array[:, i]; i+=1
        pitch = log_memory_array[:, i]; i+=1
        yaw = log_memory_array[:, i]; i+=1
        agv_x = log_memory_array[:, i]; i+=1
        agv_y = log_memory_array[:, i]; i+=1
        agv_z = log_memory_array[:, i]; i+=1
        x_ref = log_memory_array[:, i]; i+=1
        y_ref = log_memory_array[:, i]; i+=1
        z_ref = log_memory_array[:, i]; i+=1
        roll_ref = log_memory_array[:, i]; i+=1
        pitch_ref = log_memory_array[:, i]; i+=1
        yaw_ref = log_memory_array[:, i]; i+=1
        x_vel_ref = log_memory_array[:, i]; i+=1
        y_vel_ref = log_memory_array[:, i]; i+=1
        z_vel_ref = log_memory_array[:, i]; i+=1
        agv_x_ref = log_memory_array[:, i]; i+=1
        agv_y_ref = log_memory_array[:, i]; i+=1
        agv_z_ref = log_memory_array[:, i]; i+=1
        print('i=%s ' %i)

        if type == None:
            self.plot_all(timestamp, pos_x, pos_y, pos_z, x_ref, y_ref, z_ref, vel_x, vel_y, vel_z, x_vel_ref, y_vel_ref, z_vel_ref,
                roll, pitch, yaw, roll_ref, pitch_ref, yaw_ref, agv_x, agv_y, agv_z, agv_x_ref, agv_y_ref, agv_z_ref)
        elif type == 'attitude':
            self.plot_attitude(timestamp, roll, pitch, yaw, roll_ref, pitch_ref, yaw_ref, agv_x, agv_y, agv_z, agv_x_ref, agv_y_ref, agv_z_ref)
        # plt.figure(figsize=(15, 15))
        # plt.subplot(2,2,1)
        # plt.plot(timestamp, vel_x, 'r', timestamp, vel_y, 'g', timestamp, vel_z, 'b', timestamp, x_vel_ref, 'y--', timestamp, y_vel_ref, 'm--', timestamp, z_vel_ref, 'k--')
        # plt.ylabel('vel')
        # plt.grid(True)
        # plt.legend(loc='upper left')
        # plt.subplot(2,2,2)
        # plt.plot(timestamp, agv_x, 'r', timestamp, agv_y, 'g', timestamp, agv_z, 'b', timestamp, agv_x_ref, 'y--', timestamp, agv_y_ref, 'm--', timestamp, agv_z_ref, 'k--')
        # plt.ylabel('agv')
        # plt.grid(True)
        # plt.legend(loc='upper left')
        # plt.subplot(2,2,3)
        # plt.plot(timestamp, pos_x, 'r', timestamp, pos_y, 'g', timestamp, pos_z, 'b', timestamp, x_ref, 'y--', timestamp, y_ref, 'm--', timestamp, z_ref, 'k--')
        # plt.ylabel('pos')
        # plt.grid(True)
        # plt.legend(loc='upper left')
        # plt.subplot(2,2,4)
        # plt.plot(timestamp, roll, 'r', timestamp, pitch, 'g', timestamp, yaw, 'b', timestamp, roll_ref, 'y--', timestamp, pitch_ref, 'm--', timestamp, yaw_ref, 'k--')
        # plt.ylabel('rpy')
        # plt.grid(True)
        # plt.legend(loc='upper left')
		
        # plt.show()

    def plot_base(self):
        n = len(self.log_memory)
        log_memory_array = np.asarray(self.log_memory[1:n])
        timestamp = log_memory_array[:, 0] - log_memory_array[0, 0]
        i = 2
        pos_x = log_memory_array[:, i]; i+=1
        pos_y = log_memory_array[:, i]; i+=1
        pos_z = log_memory_array[:, i]; i+=1
        vel_x = log_memory_array[:, i]; i+=1
        vel_y = log_memory_array[:, i]; i+=1
        vel_z = log_memory_array[:, i]; i+=1
        roll = log_memory_array[:, i]; i+=1
        pitch = log_memory_array[:, i]; i+=1
        yaw = log_memory_array[:, i]; i+=1
        agv_x = log_memory_array[:, i]; i+=1
        agv_y = log_memory_array[:, i]; i+=1
        agv_z = log_memory_array[:, i]; i+=1
        x_ref = log_memory_array[:, i]; i+=1
        y_ref = log_memory_array[:, i]; i+=1
        z_ref = log_memory_array[:, i]; i+=1
        roll_ref = log_memory_array[:, i]; i+=1
        pitch_ref = log_memory_array[:, i]; i+=1
        yaw_ref = log_memory_array[:, i]; i+=1
        x_vel_ref = log_memory_array[:, i]; i+=1
        y_vel_ref = log_memory_array[:, i]; i+=1
        z_vel_ref = log_memory_array[:, i]; i+=1
        agv_x_ref = log_memory_array[:, i]; i+=1
        agv_y_ref = log_memory_array[:, i]; i+=1
        agv_z_ref = log_memory_array[:, i]; i+=1
        print('i=%s ' %i)
        
        fig = plt.figure(figsize=(15, 15))
        ax0 = fig.add_axes([0.05, 0.05, 0.45, 0.45])
        ax0.plot(timestamp, vel_x, 'r', timestamp, vel_y, 'g', timestamp, vel_z, 'b', timestamp, x_vel_ref, 'y--', timestamp, y_vel_ref, 'm--', timestamp, z_vel_ref, 'k--')
        ax0.ylabel('vel')
        ax0.grid(True)
        ax0.legent(loc='upper left')
        ax1 = fig.add_axes([0.55, 0.05, 0.95, 0.45])
        ax1.plot(timestamp, agv_x, 'r', timestamp, agv_y, 'g', timestamp, agv_z, 'b', timestamp, agv_x_ref, 'y--', timestamp, agv_y_ref, 'm--', timestamp, agv_z_ref, 'k--')
        ax1.ylabel('agv')
        ax1.grid(True)
        ax1.legent(loc='upper left')
        ax2 = fig.add_axes([0.05, 0.45, 0.55, 0.95])
        ax2.plot(timestamp, pos_x, 'r', timestamp, pos_y, 'g', timestamp, pos_z, 'b', timestamp, x_ref, 'y--', timestamp, y_ref, 'm--', timestamp, z_ref, 'k--')
        ax2.ylabel('pos')
        ax2.grid(True)
        ax2.legent(loc='upper left')
        ax3 = fig.add_axes([0.55, 0.55, 0.95, 0.95])
        ax3.plot(timestamp, roll, 'r', timestamp, pitch, 'g', timestamp, yaw, 'b', timestamp, roll_ref, 'y--', timestamp, pitch_ref, 'm--', timestamp, yaw_ref, 'k--')
        ax3.ylabel('rpy')
        ax3.grid(True)
        ax3.legent(loc='upper left')

        plt.show()

    def plot_all(self, timestamp, pos_x, pos_y, pos_z, x_ref, y_ref, z_ref, vel_x, vel_y, vel_z, x_vel_ref, y_vel_ref, z_vel_ref,
                roll, pitch, yaw, roll_ref, pitch_ref, yaw_ref, agv_x, agv_y, agv_z, agv_x_ref, agv_y_ref, agv_z_ref):
        plt.figure(figsize=(15, 15))
        plt.subplot(2,2,1)
        plt.plot(timestamp, vel_x, 'r', timestamp, vel_y, 'g', timestamp, vel_z, 'b', timestamp, x_vel_ref, 'y--', timestamp, y_vel_ref, 'm--', timestamp, z_vel_ref, 'k--')
        plt.ylabel('vel')
        plt.grid(True)
        plt.legend(loc='upper left')
        plt.subplot(2,2,2)
        plt.plot(timestamp, agv_x, 'r', timestamp, agv_y, 'g', timestamp, agv_z, 'b', timestamp, agv_x_ref, 'y--', timestamp, agv_y_ref, 'm--', timestamp, agv_z_ref, 'k--')
        plt.ylabel('agv')
        plt.grid(True)
        plt.legend(loc='upper left')
        plt.subplot(2,2,3)
        plt.plot(timestamp, pos_x, 'r', timestamp, pos_y, 'g', timestamp, pos_z, 'b', timestamp, x_ref, 'y--', timestamp, y_ref, 'm--', timestamp, z_ref, 'k--')
        plt.ylabel('pos')
        plt.grid(True)
        plt.legend(loc='upper left')
        plt.subplot(2,2,4)
        plt.plot(timestamp, roll, 'r', timestamp, pitch, 'g', timestamp, yaw, 'b', timestamp, roll_ref, 'y--', timestamp, pitch_ref, 'm--', timestamp, yaw_ref, 'k--')
        plt.ylabel('rpy')
        plt.grid(True)
        plt.legend(loc='upper left')
		
        plt.show()
    
    def plot_attitude(self, timestamp, roll, pitch, yaw, roll_ref, pitch_ref, yaw_ref, agv_x, agv_y, agv_z, agv_x_ref, agv_y_ref, agv_z_ref):
        plt.figure(figsize=(20, 15))
        plt.subplot(3,3,1)
        plt.plot(timestamp, roll, 'r', timestamp, roll_ref, 'b--', linewidth=2)
        plt.ylabel('roll')
        plt.grid(True)
        plt.legend(['roll', 'roll_ref'], loc='upper left')
        plt.subplot(3,3,2)
        plt.plot(timestamp, pitch, 'r', timestamp, pitch_ref, 'b--', linewidth=2)
        plt.ylabel('pitch')
        plt.grid(True)
        plt.legend(['pitch', 'pitch_ref'], loc='upper left')
        plt.subplot(3,3,3)
        plt.plot(timestamp, yaw, 'r', timestamp, yaw_ref, 'b--', linewidth=2)
        plt.ylabel('yaw')
        plt.grid(True)
        plt.legend(['yaw', 'yaw_ref'], loc='upper left')
        plt.subplot(3,3,4)
        plt.plot(timestamp, agv_x, 'r', linewidth=2)
        plt.ylabel('rollrate')
        plt.grid(True)
        plt.legend(['rollrate'], loc='upper left')
        plt.subplot(3,3,5)
        plt.plot(timestamp, agv_y, 'r', linewidth=2)
        plt.ylabel('pitchrate')
        plt.grid(True)
        plt.legend(['pitchrate'], loc='upper left')
        plt.subplot(3,3,6)
        plt.plot(timestamp, agv_z, 'r', linewidth=2)
        plt.ylabel('yawrate')
        plt.grid(True)
        plt.legend(['yawrate'], loc='upper left')
        plt.subplot(3,3,7)
        plt.plot(timestamp, agv_x_ref, 'b--', linewidth=2)
        plt.ylabel('rollrate_ref')
        plt.grid(True)
        plt.legend(['rollrate_ref'], loc='upper left')
        plt.subplot(3,3,8)
        plt.plot(timestamp, agv_y_ref, 'b--', linewidth=2)
        plt.ylabel('pitchrate_ref')
        plt.grid(True)
        plt.legend(['pitchrate_ref'], loc='upper left')
        plt.subplot(3,3,9)
        plt.plot(timestamp, agv_z_ref, 'b--', linewidth=2)
        plt.ylabel('yawrate_ref')
        plt.grid(True)
        plt.legend(['yawrate_ref'], loc='upper left')

        plt.show()

if __name__ == "__main__":
    logger = Logger()
    