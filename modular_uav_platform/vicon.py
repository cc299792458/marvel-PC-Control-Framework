# from __future__ import print_function
import threading
from vicon_dssdk import ViconDataStream
import argparse
import time
import numpy as np
from threading import Thread
import multiprocessing
from transforms3d.euler import euler2quat, euler2mat, mat2euler
from transforms3d.quaternions import quat2mat
# from utils import 
from utils import rpy2quat, quat2rpy, nparray2bin, bin2nparray
# from tcp_client import TcpClient
# from tcp_client_2 import TcpClient
# from test_write_txt import write_txt_file

print("loading host ...")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    'host', nargs='?', help="Host name, in the format of server:port", default="localhost:801")
args = parser.parse_args()


# self.streamingClient = ViconDataStream.Client()


class Vicon:
    def __init__(self):
        # Create New Vicon Clinet
        # self.client = TcpClient(ip="192.168.10.11", port=8800)
        # self.client = TcpClient(ip="192.168.10.11", port=8800)
        self.streamingClient = ViconDataStream.Client()
        self.position = np.zeros(3)
        self.rotation = np.zeros(4)
        self.rpy = np.zeros(3)
        self.position_prev = np.zeros(3)
        self.rotation_prev = np.zeros(4)
        self.velocity = np.zeros(3)
        self.rotation_rate = np.zeros(3)

        # self.position = multiprocessing.Array('f', 3)
        # self.rotation = multiprocessing.Array('f', 4)
        # self.rpy = multiprocessing.Array('f', 3)
        # self.position_prev = multiprocessing.Array('f', 3)
        # self.rotation_prev = multiprocessing.Array('f', 4)
        # self.velocity = multiprocessing.Array('f', 3)
        # self.rotation_rate = multiprocessing.Array('f', 3)

        self.update_time = time.time()

        self.loss_pkg = 0
        self.rotation_rate_prev = np.zeros(3)
        self.velocity_prev = np.zeros(3)

        # self.rotation_rate_prev = multiprocessing.Array('f', 3)
        # self.velocity_prev = multiprocessing.Array('f', 3)

        print("start\n")
        self.streamingClient.Connect(args.host)

        # Check the version
        print('Version', self.streamingClient.GetVersion())

        # Check setting the buffer size works
        self.streamingClient.SetBufferSize(1)

        # Enable all the data types
        self.streamingClient.EnableSegmentData()
        self.streamingClient.EnableMarkerData()
        self.streamingClient.EnableUnlabeledMarkerData()
        self.streamingClient.EnableMarkerRayData()
        self.streamingClient.EnableDeviceData()
        self.streamingClient.EnableCentroidData()

        self.testloss = np.array([4.65, 0, 0])

        # Report whether the data types have been enabled
        print('Segments', self.streamingClient.IsSegmentDataEnabled())
        print('Markers', self.streamingClient.IsMarkerDataEnabled())
        print('Unlabeled Markers',
              self.streamingClient.IsUnlabeledMarkerDataEnabled())
        print('Marker Rays', self.streamingClient.IsMarkerRayDataEnabled())
        print('Devices', self.streamingClient.IsDeviceDataEnabled())
        print('Centroids', self.streamingClient.IsCentroidDataEnabled())

        self.run()

    def postition_enu2ned(self, position):
        # rotate z 90 deg
        return np.array([position[1], -position[0], position[2]])

    def orientation_changeorder(self, quaternion):
        x, y, z, w = quaternion
        quaternion = np.array([w, x, y, z])
        return quaternion

    def orientation_enu2ned(self, quaternion):
        w, x, y, z = quaternion
        rotm = quat2mat([w, x, y, z])
        rotz90 = np.array([[0, 1, 0],
                           [-1, 0, 0],
                           [0, 0, 1]])
        rotmf = np.matmul(rotm, rotz90)
        w = 0.5 * ((1 + rotmf[0, 0] + rotmf[1, 1] + rotmf[2, 2]) ** (1 / 2))
        x = (rotmf[2, 1] - rotmf[1, 2]) / 4 * w
        y = (rotmf[0, 2] - rotmf[2, 0]) / 4 * w
        z = (rotmf[1, 0] - rotmf[0, 1]) / 4 * w
        quaternion = np.array([w, x, y, z])
        return quaternion

    def rpy2quan90(self, rpy):
        r, p, y = rpy
        rpy = np.array([p, -r, y])
        r, p, y = rpy

        # return rpy
        quan = euler2quat(r, p, y)
        return np.array(quan)
        # return [r, p, y]

    def rpycalc(self, quaternion):
        phi = np.arctan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                         (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])))
        theta = np.arcsin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))
        psi = np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                         (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])))
        return np.array([psi, theta, phi])

    def omega(self, quaternion, quaternion_prev, dt):
        # dq = (quaternion - quaternion_prev) / dt
        # w, x, y, z = quaternion
        # omega = 2 * np.mat([[w, x, y, z],
        #                     [-x, w, z, -y],
        #                     [-y, -z, w, x],
        #                     [-z, y, -x, w]]) * np.vstack(dq)
        # return np.asarray(omega[1:4]).reshape(-1)
        roll, pitch, yaw = quat2rpy(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        roll_prev, pitch_prev, yaw_prev = quat2rpy(quaternion_prev[0], quaternion_prev[1], quaternion_prev[2], quaternion_prev[3])
        roll_rate = (roll - roll_prev) / dt
        pitch_rate = (pitch - pitch_prev) / dt
        yaw_rate = (yaw - yaw_prev) / dt

        return np.array([roll_rate, pitch_rate, yaw_rate])


    def __dataProcessFunction(self):
        # dataProcessFunction

        self.streamingClient.SetStreamMode(
            ViconDataStream.Client.StreamMode.EClientPullPreFetch)
        previous_time = time.time()
        indexn = 0
        while True:
            HasFrame = False
            while not HasFrame:
                try:
                    self.streamingClient.GetFrame()
                    HasFrame = True
                except ViconDataStream.DataStreamException as e:
                    self.streamingClient.GetFrame()

            subjectNames = self.streamingClient.GetSubjectNames()
            for subjectName in subjectNames:
                segmentNames = self.streamingClient.GetSegmentNames(subjectName)
                # if not subjectName == "aerial_manipulator_1":
                #     continue

                if not subjectName == "marvel_zero":
                    continue

                for segmentName in segmentNames:
                    global_translation = self.streamingClient.GetSegmentGlobalTranslation(subjectName, segmentName)
                    self.position = np.array(global_translation[0])
                    self.position = self.postition_enu2ned(self.position)
                    self.position = self.position / 1000
                    # self.position[0] -= 1

                    global_rotation_quant_eu = self.streamingClient.GetSegmentGlobalRotationEulerXYZ(subjectName,
                                                                                                     segmentName)
                    self.rpystd = np.array(global_rotation_quant_eu[0])
                    self.rotation = self.rpy2quan90(self.rpystd)
                    # self.rpy = self.rpycalc(self.rotation)

            current_time = time.time()
            # print(current_time - self.update_time)

            if all((abs(self.position - self.testloss) < np.array([1e-4, 1e-4, 1e-4])).tolist()):
                print("loss detected!")
                self.position = self.position_prev
                self.rotation = self.rotation_prev
                self.velocity = self.velocity_prev
                self.rotation_rate = self.rotation_rate_prev

                # print("position", self.position)
                # print("rotation", self.rotation)
                # print("velocity", self.velocity)
                # print("rotation_rate", self.rotation_rate)
                continue

            dt = 0.005  # current_time - self.update_time
            self.velocity = (self.position - self.position_prev) / dt
            self.rotation_rate = self.omega(self.rotation, self.rotation_prev, dt)

            self.velocity = self.velocity * 0.8 + self.velocity_prev * 0.2
            self.rotation_rate = self.rotation_rate * 0.8 + self.rotation_rate_prev * 0.2
            if indexn == 0:
                self.velocity = np.array([0, 0, 0])
                self.rotation_rate = np.array([0, 0, 0])

            # if any((self.velocity - np.array([0, 0, 0]) > np.array([0.02, 0.02, 0.02])).tolist()):
            #     print(self.velocity)


            # write_txt_file("911_testvel_01.txt", self.velocity.tolist())

            if any([current_time - self.update_time > 0.017, any((abs(self.velocity) > np.array([5, 5, 5])).tolist()),
                    any((abs(self.rotation_rate) > np.array([10, 10, 10])).tolist())]):
                print("delay detected!", self.velocity, self.rotation_rate)
                # self.position = self.position_prev
                # self.rotation = self.rotation_prev
                self.velocity = self.velocity_prev
                self.rotation_rate = self.rotation_rate_prev

                # print("position", self.position)
                # print("rotation", self.rotation)
                # print("velocity", self.velocity)
                # print("rotation_rate", self.rotation_rate)
                # continue

            self.update_time = current_time
            self.rotation_rate_prev = self.rotation_rate
            self.velocity_prev = self.velocity
            self.position_prev = self.position
            self.rotation_prev = self.rotation

            # self.senddata = np.concatenate((self.position, self.velocity, self.rotation_rate, self.rotation))
            # print("send data")
            # print("pos", self.position)
            # print(self.velocity)
            # self.senddata = nparray2bin(self.senddata)
            # self.client.send(self.senddata)
            indexn += 1
            # time.sleep(0.003)

    def run(self):
        # retrieving data from Vicon system

        # self.dataProcess = multiprocessing.Process(
        #     target=self.__dataProcessFunction)
        # self.dataProcess.start()
        self.dataThread = Thread(target=self.__dataProcessFunction)
        self.dataThread.start()


if __name__ == "__main__":
    print("This is a test")
    tr = Vicon()

    # while(True):
    # print("========== position ==========")
    # print(tr.position)
    # print("========== rotation ==========")
    # print(tr.rotation)
    # print("========== rpy ==========")
    # print(tr.rpy)
    # print("========== position_prev ==========")
    # print(tr.position_prev)
    # print("========== rotation_prev ==========")
    # print(tr.rotation_prev)
    # print("========== velocity ==========")
    # print(tr.velocity)
    # print("========== rotation_rate ==========")
    # print(tr.rotation_rate)
    # print("")
    # time.sleep(0.5)
