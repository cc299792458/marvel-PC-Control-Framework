# from __future__ import print_function
import threading
from vicon_dssdk import ViconDataStream
import argparse
import time
import numpy as np
from threading import Thread
import multiprocessing

print("loading host ...")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    'host', nargs='?', help="Host name, in the format of server:port", default="localhost:801")
args = parser.parse_args()

# self.streamingClient = ViconDataStream.Client()


class Vicon:
    def __init__(self):
        # Create New Vicon Clinet
        self.streamingClient = ViconDataStream.Client()
        # self.position = np.zeros(3)
        # self.rotation = np.zeros(4)
        # self.rpy = np.zeros(3)
        # self.position_prev = np.zeros(3)
        # self.rotation_prev = np.zeros(4)
        # self.velocity = np.zeros(3)
        # self.rotation_rate = np.zeros(3)

        self.position = multiprocessing.Array('f', 3)
        self.rotation = multiprocessing.Array('f', 4)
        self.rpy = multiprocessing.Array('f', 3)
        self.position_prev = multiprocessing.Array('f', 3)
        self.rotation_prev = multiprocessing.Array('f', 4)
        self.velocity = multiprocessing.Array('f', 3)
        self.rotation_rate = multiprocessing.Array('f', 3)

        self.update_time = time.time()

        self.loss_pkg = 0
        # self.rotation_rate_prev = np.zeros(3)
        # self.velocity_prev = np.zeros(3)

        self.rotation_rate_prev = multiprocessing.Array('f', 3)
        self.velocity_prev = multiprocessing.Array('f', 3)

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

        # Report whether the data types have been enabled
        print('Segments', self.streamingClient.IsSegmentDataEnabled())
        print('Markers', self.streamingClient.IsMarkerDataEnabled())
        print('Unlabeled Markers',
              self.streamingClient.IsUnlabeledMarkerDataEnabled())
        print('Marker Rays', self.streamingClient.IsMarkerRayDataEnabled())
        print('Devices', self.streamingClient.IsDeviceDataEnabled())
        print('Centroids', self.streamingClient.IsCentroidDataEnabled())

        # self.run()

    def postition_enu2ned(self, position):
        # rotate y 180 deg
        return np.array([-position[0], position[2], position[1]])

    def orientation_enu2ned(self, quaternion):
		# Change order from xyzw to wxyz
        return np.array([quaternion[3], -quaternion[0], quaternion[2], quaternion[1]])

    def rpycalc(self, quaternion):
        phi = np.arctan2(2*(quaternion[0]*quaternion[3]+quaternion[1]*quaternion[2]),(1-2*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3])))
        theta = np.arcsin(2*(quaternion[0]*quaternion[2]-quaternion[3]*quaternion[1]))
        psi = np.arctan2(2*(quaternion[0]*quaternion[1]+quaternion[2]*quaternion[3]),(1-2*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2])))
        return np.array([phi, theta, psi])

    def omega(self, quaternion, quaternion_prev, dt):
        dq = (quaternion - quaternion_prev) / dt
        w, x, y, z = quaternion
        omega = 2 * np.mat([[w, x, y, z],
        					[-x, w, z, -y],
        					[-y, -z, w, x],
        					[-z, y, -x, w]]) * np.vstack(dq)
        return np.asarray(omega[1:4]).reshape(-1)

    def get_pose(self):
        self.streamingClient.SetStreamMode(
        ViconDataStream.Client.StreamMode.EClientPullPreFetch)

        HasFrame = False
        while not HasFrame:
            try:
                self.streamingClient.GetFrame()
                HasFrame = True
            except ViconDataStream.DataStreamException as e:
                self.streamingClient.GetFrame()

        # Try setting the different stream modes
        # self.streamingClient.SetStreamMode(
        #     ViconDataStream.Client.StreamMode.EClientPull)
        # print('Get Frame Pull', self.streamingClient.GetFrame(),
        #       self.streamingClient.GetFrameNumber())
        
        subjectNames = self.streamingClient.GetSubjectNames()
        for subjectName in subjectNames:
            # print(subjectName)
            segmentNames = self.streamingClient.GetSegmentNames(subjectName)
            for segmentName in segmentNames:
                # print(segmentName, 'has global translation',
                #       self.streamingClient.GetSegmentGlobalTranslation(subjectName, segmentName))
                global_translation = self.streamingClient.GetSegmentGlobalTranslation(subjectName, segmentName)
                # print(type(global_translation[0][0]))

                # for i in range(3):
                #     self.position[i] = global_translation[0][i] 
                self.position = np.array(global_translation[0])
                # change unit from milimeter to meterW
                self.position /= 1000
                # print(self.position)

                # print(segmentName, 'has global rotation( helical )',
                #       self.streamingClient.GetSegmentGlobalRotationHelical(subjectName, segmentName))
                # print(segmentName, 'has global rotation( EulerXYZ )',
                #       self.streamingClient.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName))
                # print(segmentName, 'has global rotation( Quaternion )',
                #       self.streamingClient.GetSegmentGlobalRotationQuaternion(subjectName, segmentName))
                global_rotation_quat = self.streamingClient.GetSegmentGlobalRotationQuaternion(subjectName, segmentName)

                # for i in range(4):
                #     self.rotation[i] = global_rotation_quat[0][i]
                self.rotation = np.array(global_rotation_quat[0])
                # print("get quan")
                # print(self.rotation)

                # print(segmentName, 'has global rotation( Matrix )',
                #       self.streamingClient.GetSegmentGlobalRotationMatrix(subjectName, segmentName))

            # self.position = self.postition_enu2ned(self.position) # modify crazyflie's position message from motion capturing system
            # self.rotation = self.orientation_enu2ned(self.rotation)
            # self.rpy = self.rpycalc(self.rotation) # not used
            
            # current_time = time.time()
            # print(current_time-self.update_time)
            # dt = 0.01 #current_time - self.update_time
            # self.velocity = (self.position - self.position_prev) / dt
            # # print("**************")
            # # print(self.position)
            # # print(self.position_prev)
            # # print(self.position - self.position_prev)
            # self.rotation_rate = self.omega(self.rotation, self.rotation_prev, dt)

            # self.velocity = self.velocity*0.9 + self.velocity_prev*0.1
            # self.rotation_rate = self.rotation_rate*0.9 + self.rotation_rate_prev*0.1

            # if self.loss_pkg == 1:
            #     if (self.position != self.position_prev).any() and (self.rotation != self.rotation_prev).any():
            #         self.velocity = self.velocity_prev
            #         self.rotation_rate = self.rotation_rate_prev
            #         self.loss_pkg = 0
            #         print('?')
            #     else:
            #         if (self.position == self.position_prev).all() or (self.rotation == self.rotation_prev).all():
            #             self.loss_pkg = 1
            #             print('!')
            #         else:
            #             self.rotation_rate_prev = self.rotation_rate
            #             self.velocity_prev = self.velocity

            # self.update_time = current_time
            # self.position_prev = self.position
            # self.rotation_prev = self.rotation

            return self.position, self.rotation




    def __dataProcessFunction(self):
        # dataProcessFunction

        self.streamingClient.SetStreamMode(
        ViconDataStream.Client.StreamMode.EClientPullPreFetch)
        indexn = 0
        while(True):
            HasFrame = False
            while not HasFrame:
                try:
                    self.streamingClient.GetFrame()
                    HasFrame = True
                except ViconDataStream.DataStreamException as e:
                    self.streamingClient.GetFrame()

            # Try setting the different stream modes
            # self.streamingClient.SetStreamMode(
            #     ViconDataStream.Client.StreamMode.EClientPull)
            # print('Get Frame Pull', self.streamingClient.GetFrame(),
            #       self.streamingClient.GetFrameNumber())
        



            subjectNames = self.streamingClient.GetSubjectNames()
            for subjectName in subjectNames:
                # print(subjectName)
                segmentNames = self.streamingClient.GetSegmentNames(subjectName)
                for segmentName in segmentNames:
                    # print(segmentName, 'has global translation',
                    #       self.streamingClient.GetSegmentGlobalTranslation(subjectName, segmentName))
                    global_translation = self.streamingClient.GetSegmentGlobalTranslation(subjectName, segmentName)
                    # print(type(global_translation[0][0]))

                    # for i in range(3):
                    #     self.position[i] = global_translation[0][i] 
                    self.position = np.array(global_translation[0])/ 1000

                    # print(segmentName, 'has global rotation( helical )',
                    #       self.streamingClient.GetSegmentGlobalRotationHelical(subjectName, segmentName))
                    # print(segmentName, 'has global rotation( EulerXYZ )',
                    #       self.streamingClient.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName))
                    # print(segmentName, 'has global rotation( Quaternion )',
                    #       self.streamingClient.GetSegmentGlobalRotationQuaternion(subjectName, segmentName))
                    global_rotation_quat = self.streamingClient.GetSegmentGlobalRotationQuaternion(subjectName, segmentName)

                    # for i in range(4):
                    #     self.rotation[i] = global_rotation_quat[0][i]
                    self.rotation = np.array(global_rotation_quat[0])

                    # print(segmentName, 'has global rotation( Matrix )',
                    #       self.streamingClient.GetSegmentGlobalRotationMatrix(subjectName, segmentName))

            # self.position = self.postition_enu2ned(self.position) # modify crazyflie's position message from motion capturing system
            # self.rotation = self.orientation_enu2ned(self.rotation)
            # self.rpy = self.rpycalc(self.rotation) # not used
            
            current_time = time.time()
            # print(current_time-self.update_time)
            dt = 0.005 #current_time - self.update_time
            self.velocity = (self.position - self.position_prev) / dt
            # print("**************")
            # print(self.position)
            # print(self.position_prev)
            # print(self.position - self.position_prev)
            self.rotation_rate = self.omega(self.rotation, self.rotation_prev, dt)

            self.velocity = self.velocity*0.9 + self.velocity_prev*0.1
            self.rotation_rate = self.rotation_rate*0.9 + self.rotation_rate_prev*0.1
            if indexn == 0:
                self.velocity = np.array([0, 0, 0])
                self.rotation_rate = np.array([0, 0, 0])

            if any([current_time - self.update_time > 0.0085, any((abs(self.velocity) > np.array([10, 10, 10])).tolist()),
                    any((abs(self.rotation_rate) > np.array([10, 10, 10])).tolist())]):
                print("delay detected!", self.velocity, self.rotation_rate)
                # self.position = self.position_prev
                # self.rotation = self.rotation_prev
                self.velocity = self.velocity_prev
                self.rotation_rate = self.rotation_rate_prev



            self.update_time = current_time
            self.rotation_rate_prev = self.rotation_rate
            self.velocity_prev = self.velocity
            self.position_prev = self.position
            self.rotation_prev = self.rotation
            indexn += 1
    def run(self):
        # retrieving data from Vicon system

        # self.dataProcess = multiprocessing.Process(
        #     target=self.__dataProcessFunction)
        # self.dataProcess.start()
        self.dataThread = multiprocessing.Process(target=self.__dataProcessFunction)
        self.dataThread.start()

    

    
        

if __name__ == "__main__":

    print("This is a test")
    vicon = Vicon()

    while(True):
        [pos, rot] = vicon.get_pose()
        # print("========== position ==========")
        # print(vicon.position)
        # print("========== rotation ==========")
        # print(vicon.rotation)
        # print("========== position_prev ==========")
        # print(vicon.position_prev)
        # print("========== rotation_prev ==========")
        # print(vicon.rotation_prev)
        # print("========== velocity ==========")
        # print(vicon.velocity)
        # print("========== rotation_rate ==========")
        # print(vicon.rotation_rate)
        print("========== position ==========")
        print(pos)
        print("========== rotation ==========")
        print(rot)
        print("")
        #time.sleep(1)