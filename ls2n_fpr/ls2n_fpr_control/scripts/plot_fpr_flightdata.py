#!/usr/bin/env python3
import os
import csv
import pickle
from tkinter import Tk
from tkinter import filedialog
import matplotlib.pyplot as plt
import matplotlib.patches as mpl_patches
import numpy as np
import transforms3d as tf3d
from ament_index_python.packages import get_package_share_directory
from math import sqrt
from sklearn.metrics import mean_squared_error

Traj_File = 'fpr_hovering' # trajectory file name

class FprFlightData():
    def __init__(self, logfile):
        self.raw_data = {}
        self.topic_names = []
        self.read_csv_data(logfile)
        if not 'FPR.Traj' in self.topic_names:
            self.read_trajectory_data() # not used for latest log files
        self.regulate_timestamp()
        self.parse_fpr_pose()
        self.parse_drone_attitude()
        self.plot_fpr_data()
        self.plot_drone_data()
        # self.save_error_data()

    def read_csv_data(self, logfile):
        with open(logfile, 'r') as file:
            reader = csv.reader(file, delimiter=';')
            idx_row = 0
            for row in reader:
                idx_row = idx_row+1
                if idx_row == 1:
                    self.topic_names = row
                    print('Topic Names: ', row)
                    for name in self.topic_names:
                        self.raw_data[name] = []
                else:
                    for i in range(len(row)):
                        vec = self.str_to_vec(row[i])
                        if np.size(self.raw_data[self.topic_names[i]], axis=0) == 0:
                            self.raw_data[self.topic_names[i]] = vec
                        else:
                            self.raw_data[self.topic_names[i]] = np.append(self.raw_data[self.topic_names[i]], vec, axis=0)

    def str_to_vec(self, str):
        str = str[1:-1]
        vec = []
        last_idx = 0
        for i in range(len(str)):
            if str[i] == ";":
                str_temp = str[last_idx:i]
                last_idx = i+1
                vec = np.append(vec, float(str_temp))
        return np.reshape(vec,(1,np.size(vec)))
            
    def read_trajectory_data(self):
        filename = os.path.join(get_package_share_directory('ls2n_tools'),
                                'trajectories', Traj_File + ".traj.pickle")
        try:
            file = open(filename, 'rb')
        except FileNotFoundError:
            exit("Trajectory file %s does not exist, please generate with ls2n_tools package.", str(Traj_File))
        raw_trajectory = pickle.load(file)
        variables = list(raw_trajectory.keys())
        self.trajectory = []
        pos_raw = np.zeros((1,9))
        vel_raw = np.zeros((1,9))
        accel_raw = np.zeros((1,9))
        for time_index in range(len(raw_trajectory["time"])):
            var_index = 0
            timestamp = raw_trajectory["time"][time_index]
            for var in variables:
                if var != "time" and var[-1] != "D":
                    pos_raw[0,var_index] = raw_trajectory[var][time_index]
                    if var+"D" in raw_trajectory:
                        vel_raw[0,var_index] = raw_trajectory[var+"D"][time_index]
                    if var+"DD" in raw_trajectory:
                        accel_raw[0,var_index] = raw_trajectory[var+"DD"][time_index]
                    var_index += 1
            traj = np.concatenate((pos_raw, vel_raw, accel_raw),axis=1)
            traj_timestamped = np.array([[timestamp]])
            traj_timestamped = np.append(traj_timestamped, traj, axis=1)
            if time_index == 0:
                self.trajectory = traj_timestamped
            else:
                self.trajectory = np.append(self.trajectory, traj_timestamped, axis=0)

    def regulate_timestamp(self):
        start_time = self.raw_data[self.topic_names[0]][0,0]
        for name in self.topic_names:
            start_time = min(start_time, self.raw_data[name][0,0])
        for name in self.topic_names:
            self.raw_data[name][:,0] = (self.raw_data[name][:,0] - start_time)

    def parse_fpr_pose(self):
        self.fpr_pose_raw = self.raw_data['FPR.Pose']
        self.fpr_pose = np.zeros((np.size(self.fpr_pose_raw,axis=0),10))
        self.fpr_pose[:,0:4] = self.fpr_pose_raw[:,0:4]
        quaternion_fpr = np.append(self.fpr_pose_raw[:,7].reshape(-1,1), self.fpr_pose_raw[:,4:7], axis=1)
        self.fpr_pose[:,4:7] = self.quat2euler(quaternion_fpr)
        self.fpr_pose[:,7:10] = self.fpr_pose_raw[:,8:11]
        
        if 'FPR.Traj' in self.topic_names:
            self.trajectory_raw = self.raw_data['FPR.Traj']
            self.trajectory = np.zeros((np.size(self.trajectory_raw,axis=0),10))
            self.trajectory[:,0:4] = self.trajectory_raw[:,0:4]
            quaternion_traj = np.append(self.trajectory_raw[:,7].reshape(-1,1), self.trajectory_raw[:,4:7], axis=1)
            self.trajectory[:,4:7] = self.quat2euler(quaternion_traj)
            self.trajectory[:,7:10] = self.trajectory_raw[:,8:11]
        else:
            self.trajectory[:,1:4] += self.fpr_pose[0,1:4]
    
    def parse_drone_attitude(self):
        self.drone_odom_raw = {}
        self.drone_odom = {}
        self.drone_setpoint_raw = {}
        self.drone_setpoint = {}
        for name in self.topic_names:
            if 'Drone' in name and 'Odom' in name:
                self.drone_odom_raw[name] = self.raw_data[name]
                self.drone_odom[name] = np.zeros((np.size(self.drone_odom_raw[name], axis=0),7))
                self.drone_odom[name][:,0:4] = self.drone_odom_raw[name][:,0:4]
                self.drone_odom[name][:,4:7] = self.quat2euler(self.drone_odom_raw[name][:,4:8])
            if 'Drone' in name and 'Setpoint' in name:
                self.drone_setpoint_raw[name] = self.raw_data[name]
                self.drone_setpoint[name] = np.zeros((np.size(self.drone_setpoint_raw[name], axis=0),5))
                self.drone_setpoint[name][:,0:2] = self.drone_setpoint_raw[name][:,0:2]
                self.drone_setpoint[name][:,2:5] = self.quat2euler(self.drone_setpoint_raw[name][:,2:6])

    def quat2euler(self, quat):
        size = np.size(quat, axis=0)
        if size == 0:
            return
        elif size == 1:
            eul = tf3d.euler.quat2euler(quat)
        else:
            eul = np.zeros((size,3))
            for i in range(size):
                eul[i,:] = tf3d.euler.quat2euler(quat[i,:])
        return eul

    def plot_fpr_data(self):
        traj_size = np.size(self.trajectory, 0)
        pose_size = np.size(self.fpr_pose, 0)
        
        handles = [mpl_patches.Rectangle((0, 0), 1, 1, fc="white", ec="white", 
                                 lw=0, alpha=0)]*2

        plt.figure("Platform Position")
        title = ['Xp', 'Yp', 'Zp']
        for i in range(3):
            plt.subplot(1,3,i+1)
            plt.title(title[i])
            plt.plot(self.fpr_pose[:,0], self.fpr_pose[:,i+1])
            plt.plot(self.trajectory[:,0], self.trajectory[:,i+1])
            if traj_size == pose_size:
                rms = sqrt(mean_squared_error(self.trajectory[:,i+1], self.fpr_pose[:,i+1]))
                labels=["rms error: {0:0.4g}".format(rms)]
                plt.legend(handles, labels, loc='best', fontsize='small', fancybox=True, framealpha=0.7, 
                    handlelength=0, handletextpad=0)
            plt.grid()

        plt.figure("Platform Orientation")
        title = ['Roll', 'Pitch', 'Yaw']
        for i in range(3):
            plt.subplot(1,3,i+1)
            plt.title(title[i])
            plt.plot(self.fpr_pose[:,0], self.fpr_pose[:,i+4])
            plt.plot(self.trajectory[:,0], self.trajectory[:,i+4])
            if traj_size == pose_size:
                rms = sqrt(mean_squared_error(self.trajectory[:,i+4], self.fpr_pose[:,i+4]))
                labels=["rms error: {0:0.4g}".format(rms)]
                plt.legend(handles, labels, loc='best', fontsize='small', fancybox=True, framealpha=0.7, 
                    handlelength=0, handletextpad=0)
            plt.grid()

        plt.figure("Leg Angles")
        title = ['Leg 1', 'Leg 2', 'Leg 3']
        for i in range(3):
            plt.subplot(1,3,i+1)
            plt.title(title[i])
            plt.plot(self.fpr_pose[:,0], self.fpr_pose[:,i+7])
            plt.plot(self.trajectory[:,0], self.trajectory[:,i+7])
            if traj_size == pose_size:
                rms = sqrt(mean_squared_error(self.trajectory[:,i+7], self.fpr_pose[:,i+7]))
                labels=["rms error: {0:0.4g}".format(rms)]
                plt.legend(handles, labels, loc='best', fontsize='small', fancybox=True, framealpha=0.7, 
                    handlelength=0, handletextpad=0)
            plt.grid()

        # if 'FPR.AuxControl' in self.topic_names:
        #     self.aux_control = self.raw_data['FPR.AuxControl']
        #     plt.figure("Auxiliary Control Outputs")
        #     title = ['Position', 'Orientation', 'Legs']
        #     for i in range(3):
        #         plt.subplot(1,3,i+1)
        #         plt.title(title[i])
        #         plt.plot(self.aux_control[:,0], self.aux_control[:,(i*3+1):(i*3+4)])
        #         if i!=2:
        #             plt.legend(('x','y','z'))
        #         else:
        #             plt.legend(('Leg 1','Leg 2','Leg 3'))
        #         plt.grid()
        
        if 'FPR.ExternalWrench' in self.topic_names:
            self.external_wrench = self.raw_data['FPR.ExternalWrench']
            plt.figure("External Wrench Estimation")
            title = ['Position', 'Orientation', 'Legs']
            for i in range(3):
                plt.subplot(1,3,i+1)
                plt.title(title[i])
                plt.plot(self.external_wrench[:,0], self.external_wrench[:,(i*3+1):(i*3+4)])
                if i!=2:
                    plt.legend(('x','y','z'))
                else:
                    plt.legend(('Leg 1','Leg 2','Leg 3'))
                plt.grid()
        
        # if 'FPR.ExternalWrenchFpr' in self.topic_names:
        #     self.external_wrench_fpr = self.raw_data['FPR.ExternalWrenchFpr']
        #     plt.figure("External Wrench Estimation for Fpr Interaction")
        #     title = ['Position', 'Orientation', 'Legs']
        #     for i in range(3):
        #         plt.subplot(1,3,i+1)
        #         plt.title(title[i])
        #         plt.plot(self.external_wrench_fpr[:,0], self.external_wrench_fpr[:,(i*3+1):(i*3+4)])
        #         if i!=2:
        #             plt.legend(('x','y','z'))
        #         else:
        #             plt.legend(('Leg 1','Leg 2','Leg 3'))
        #         plt.grid()
        
        # if 'FPR.ExternalWrenchbyDrones' in self.topic_names:
        #     self.external_wrench_drone = self.raw_data['FPR.ExternalWrenchbyDrones']
        #     plt.figure("External Wrench Caused by Drones")
        #     title = ['Position', 'Orientation', 'Legs']
        #     for i in range(3):
        #         plt.subplot(1,3,i+1)
        #         plt.title(title[i])
        #         plt.plot(self.external_wrench_drone[:,0], self.external_wrench_drone[:,(i*3+1):(i*3+4)])
        #         if i!=2:
        #             plt.legend(('x','y','z'))
        #         else:
        #             plt.legend(('Leg 1','Leg 2','Leg 3'))
        #         plt.grid()
        if 'FPR.DesiredForce' in self.topic_names:
            self.desired_force = self.raw_data['FPR.DesiredForce']
            plt.figure("Desired Contact Force")
            plt.plot(self.desired_force[:,0], self.desired_force[:,1])
            plt.legend('F')
            plt.grid()

    def plot_drone_data(self):
        drone_names = ['Drone1', 'Drone2', 'Drone3']
        for name in drone_names:
            plt.figure("Attitude "+name)
            title = ['Roll', 'Pitch', 'Yaw']
            for i in range(3):
                plt.subplot(1,3,i+1)
                plt.title(title[i])
                plt.plot(self.drone_odom[name+'.Odom'][:,0], self.drone_odom[name+'.Odom'][:,i+4])
                plt.plot(self.drone_setpoint[name+'.Setpoint'][:,0], self.drone_setpoint[name+'.Setpoint'][:,i+2])
                plt.grid()
        
        plt.figure("Desired Thrusts")
        title = ['Drone1', 'Drone2', 'Drone3']
        for i in range(3):
            plt.subplot(1,3,i+1)
            plt.title(title[i])
            plt.plot(self.drone_setpoint[drone_names[i]+'.Setpoint'][:,0], self.drone_setpoint[drone_names[i]+'.Setpoint'][:,1])
            plt.grid()

        if 'Drone.ExternalForce' in self.topic_names:
            self.drone_external_force = self.raw_data['Drone.ExternalForce']
            plt.figure("Drone External Force Estimation")
            title = ['Drone 1 External Force', 'Drone 2 External Force', 'Drone 3 External Force']
            for i in range(3):
                plt.subplot(1,3,i+1)
                plt.title(title[i])
                plt.plot(self.drone_external_force[:,0], self.drone_external_force[:,(i*3+1):(i*3+4)])
                plt.legend(('fx','fy','fz'))
                plt.grid()
    
    def save_error_data(self):
        self.error = self.trajectory[:,0:10] - self.fpr_pose[:,0:10]
        self.error[:,0] = self.trajectory[:,0]
        np.savetxt('error.csv', self.error, delimiter=';')

def main():
    print("Selecting csv file...")
    root = Tk()
    root.withdraw()
    root.filename = filedialog.askopenfilename(initialdir = "/home/user/logs/fpr")
    print("File selected: ", root.filename)
    
    fprData = FprFlightData(root.filename)
    root.destroy()

    while(True):
        try:
            plt.show()
        except KeyboardInterrupt:
            print("Interrupt by user, closing all windows...")
        finally:
            plt.close()
            exit(1)

if __name__ == '__main__':
    main()
