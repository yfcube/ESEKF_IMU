import numpy as np
import matplotlib.pyplot as plt
import transformations as tr
import yaml
import math
from esekf import *


class DataProcessor(object):
    def __init__(self, imu_data: np.array, gt_data: np.array):
       self.imu_format_size = 7
       self.gt_format_size = 11
       self.imu_idx = 0
       self.gt_idx = 0 
       self.IMU_END = imu_data.shape[0]
       self.GT_END = gt_data.shape[0]
       self.imu_data = imu_data
       self.gt_data = gt_data
       self.last_imu = np.zeros(self.imu_format_size)
       self.last_gt = np.zeros(self.gt_format_size)

       self.last_imu = self.imu_data[0,:]
       self.last_gt = self.gt_data[0,:]
       print(self.imu_idx)
       print(self.gt_idx)
       print("imu data size: ", self.IMU_END)
       print("gt data size: ", self.GT_END)

    def getLatestData(self):   
        a = int(self.imu_idx < self.IMU_END) 
        b = int(self.gt_idx < self.GT_END)
        c = a*2+b ## a<<2 +b, how to write this in python
        if c==0b11:
            if self.imu_data[self.imu_idx,0] < self.gt_data[self.gt_idx,0]:
                data = self.imu_data[self.imu_idx,:]
                self.imu_idx = self.imu_idx + 1
                return True, 1, data
            else:
                data = self.gt_data[self.gt_idx,:]
                self.gt_idx= self.gt_idx + 1
                return True, 2, data
        elif c==0b10:
            data = self.imu_data[self.imu_idx,:]
            self.imu_idx = self.imu_idx + 1
            return True, 1, data
        elif c==0b01:
            data = self.gt_data[self.gt_idx,:]
            self.gt_idx = self.gt_idx + 1
            return True, 2, data 
        else: # c==0b00
            return False, 0, np.zeros(self.imu_format_size)

def load_imu_parameters():
    f = open('./data/params.yaml', 'r')
    yml = yaml.load(f.read())
    params = ImuParameters()
    params.frequency = yml['IMU.frequency']
    params.sigma_a_n = yml['IMU.acc_noise_sigma']  # m/sqrt(s^3)
    params.sigma_w_n = yml['IMU.gyro_noise_sigma']  # rad/sqrt(s)
    params.sigma_a_b = yml['IMU.acc_bias_sigma']     # m/sqrt(s^5)
    params.sigma_w_b = yml['IMU.gyro_bias_sigma']    # rad/sqrt(s^3)
    f.close()
    return params

def main():
    imu_data = np.loadtxt('./matlab/imudata.txt')
    gt_data = np.loadtxt('./matlab/gtdata.txt')
    latest_gt_data = gt_data[0,:] # just want init this variable

    data_process = DataProcessor(imu_data, gt_data)
    
    imu_parameters = load_imu_parameters()

    init_nominal_state = np.zeros((19,))
    init_nominal_state[:10] = gt_data[0, 1:]                # init p, q, v; q=[w,x,y,z]
    print("init state: ", init_nominal_state)
    init_nominal_state[10:13] = 0                           # init ba
    init_nominal_state[13:16] = 0                           # init bg
    init_nominal_state[16:19] = np.array([0, 0, -9.79])     # init g
    estimator = ESEKF(init_nominal_state, imu_parameters, gt_data[0,0])

    traj_est = [gt_data[0, :8]]
    update_ratio = 3    # control the frequency of ekf updating.
    sigma_measurement_p = 0.02   # in meters
    sigma_measurement_q = 0.001  # in rad
    # sigma_measurement = np.eye(6)
    # sigma_measurement[0:3, 0:3] *= sigma_measurement_p**2
    # sigma_measurement[3:6, 3:6] *= sigma_measurement_q**2

    sigma_measurement = np.eye(7)
    sigma_measurement[0:3, 0:3] *= sigma_measurement_p**2
    sigma_measurement[3:7, 3:7] *= sigma_measurement_q**2

    go_on = True
    while go_on:
        state, data_type, data = data_process.getLatestData()
        if not state:
            go_on = False
            continue
        if data_type==1:
            # print("Preditction at ", data[0])
            # last_preditct_time = data[0]
            estimator.predict(data)
        elif data_type==2:
            latest_gt_data = data
        
        last_preditct_time = estimator.getLastPreditcTime()
        if latest_gt_data[0]>0.1 and (abs(last_preditct_time-latest_gt_data[0])<0.005) :
            estimator.update_legacy(latest_gt_data[1:8], sigma_measurement) 

        # print('[%f]:' % last_preditct_time, estimator.nominal_state)
        frame_pose = np.zeros(8,)
        frame_pose[0] = last_preditct_time
        frame_pose[1:] = estimator.nominal_state[:7]
        traj_est.append(frame_pose)
    
    # save trajectory to TUM format
    traj_est = np.array(traj_est)
    np.savetxt('./data/traj_gt_out.txt', gt_data[:, :8])
    np.savetxt('./data/traj_esekf_out.txt', traj_est)

if __name__ == '__main__':
    main()
