"""
Example configurations for the sensor types.
Positions and orientations must still be set.

"""
from __future__ import division
import numpy as np
from copy import deepcopy

##################################################

accelerometer = {
    "type": "accelerometer",
    "freq": 250,  # sampling rate, Hz
    "pos": None,  # position on the craft (body)
    "rotmat": None,  # orientation matrix relative to the craft (sensor in body)
    "noise_covar": np.diag(np.square([0.02, 0.02, 0.02])).tolist(),  # noise covariance matrix
    "bias0": [0.04, 0.02, -0.008],  # initial bias
    "bias_taus": [1, 1, 1],  # bias random-walk return time constants
    "bias_covar": np.diag(np.square([0.0001, 0.0001, 0.0001])).tolist(),  # bias random-walk covariance matrix
    "thermal_gain": 0  # scale factor for temperature deviation's effect on noise
}

##################################################

gyroscope = {
    "type": "gyroscope",
    "freq": 250,
    "pos": None,
    "rotmat": None,
    "noise_covar": np.diag(np.square([0.008, 0.008, 0.008])).tolist(),  # rad/s
    "bias0": [0.05, -0.07, 0.004],
    "bias_taus": [1, 1, 1],
    "bias_covar": np.diag(np.square([0.0001, 0.0001, 0.0001])).tolist(),
    "thermal_gain": 0
}

##################################################

imu = {
    "type": "imu",
    "freq": 250,  # must match the freq parameters internal to gyro and accel
    "output_downsample_ratio": 5,
    "integrate_sensors": True,
    "use_integrated_units": True,
    "gyro": deepcopy(gyroscope),
    "accel": deepcopy(accelerometer)
}

##################################################

magnetometer = {
    "type": "magnetometer",
    "freq": 100,
    "pos": None,
    "rotmat": None,
    "noise_covar": np.diag(np.square([2e-6, 2e-6, 2e-6])).tolist(),  # T
    "hard_distort": [1e-12, 1e-12, 1e-12],  # "hard-iron" distortion vector
    "soft_distort": np.diag([1.0, 1.0, 1.0]).tolist()  # "soft-iron" distortion matrix
}

##################################################

barometer = {
    "type": "barometer",
    "freq": 10,
    "pos": None,
    "noise_stdev": 6,  # Pa
    "bias": 0.08  # Pa
}

##################################################

thermometer = {
    "type": "thermometer",
    "freq": 10,
    "pos": None,
    "noise_stdev": 0.5,  # K
    "bias": 0.01  # K
}

##################################################

pitot = {
    "type": "pitot",
    "freq": 10,
    "pos": None,
    "dir": None,  # direction of positive measurement (body)
    "noise_stdev": 3,  # Pa
    "bias": -0.005  # Pa
}

##################################################

gps = {
    "type": "gps",
    "freq": 10,
    "pos": None,
    "noise_covar": np.diag([1e-12]*6).tolist(),  # corresponds to [lat, lon, alt, vel_east, vel_north, vel_up]
    "bias0": [0]*6,  # [deg, deg, m, m/s, m/s, m/s]
    "bias_taus": [1]*6,
    "bias_covar": np.diag([1e-12]*6).tolist()
}

##################################################

rc_channels = {
    "type": "rc_channels",
    "num_channels": 18,
    "freq": 80,
    "default_channel_value": 1200.0,
    "pos": None
}
