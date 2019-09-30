"""
Baseline example model for a delta-winged aircraft.
Body initial conditions must still be set.
(See example_airplane for more detailed parameter comments).

"""
from __future__ import division
import numpy as np
from copy import deepcopy

##################################################

com = np.array([12, 0.0, -25])*1e-3

body = {
    "pos0": None,
    "ori0": None,
    "vel0": None,
    "avel0": None,
    "mass": 2,
    "inertia": (np.array([[84314190,  -311527,   -780425],
                          [ -311527, 61676423,     16117],
                          [ -780425,    16117, 143994168]])*1e-9).tolist(),
    "centroid": (np.array([11.5, 0, -25])*1e-3 - com).tolist(),
    "bounding_box": [0.654, 1.95, 0.2],
    "areas": (np.array([47267, 52903, 501768])*1e-6).tolist(),
    "volume": 15683576*1e-10,
    "cenpres": (np.array([11.5, 0, -25])*1e-3 - com).tolist(),
    "Cp1": np.diag([2.0, 7, 14]).tolist(),
    "Cp2": np.diag([0.5, 2, 4]).tolist(),
    "Cp2": [[  0.5, 0.0, -1e-5],
            [  0.0, 0.5,   0.0],
            [-1e-5, 0.0,   2.0]],
    "Cq": np.diag([0.2, 0.03, 0.03]).tolist(),
    "bounciness": 0.001
}

##################################################

thruster = {
    "type": "thruster",
    "dia": 0.178,
    "pos": (np.array([-282, 0, -27])*1e-3 - com).tolist(),
    "dir": [1, 0, 0],
    "c1": 1e-10,
    "c2": 3e-7,
    "kr": -7e-6,
    "kv": 6e-5,
    "rpm_min": 0,
    "rpm_max": 900*10,
    "deadband": 100,
    "rpmdot": 30000,
    "power_max": 200
}

lail = {
    "type": "surface",
    "dims": [-0.06, 0.41],
    "coeff": 3,
    "origin": (np.array([-158, 450, -14.5])*1e-3 - com).tolist(),
    "orient0": {"w": 0.9990482, "x": 0, "y": 0, "z": 0.0436194},
    "cenpres": [-0.06/2, 0.41/2, 0],
    "axis": [0, -1, 0],
    "ang_min": -15,
    "ang_max": 15,
    "angdot": 180
}

rail = {
    "type": "surface",
    "dims": [-0.06, -0.41],
    "coeff": 3,
    "origin": (np.array([-158, -450, -14.5])*1e-3 - com).tolist(),
    "orient0": {"w": 0.9990482, "x": 0, "y": 0, "z": -0.0436194},
    "cenpres": [-0.06/2, -0.41/2, 0],
    "axis": [0, 1, 0],
    "ang_min": -15,
    "ang_max": 15,
    "angdot": 180
}

lfin = {
    "type": "surface",
    "dims": [-0.247, 0.130],
    "coeff": 2,
    "origin": (np.array([0, 190, 0])*1e-3 - com).tolist(),
    "orient0": {"w": float(np.sqrt(2)/2), "x": float(np.sqrt(2)/2), "y": 0, "z": 0},
    "cenpres": [-0.247/2, 0.130/2, 0],
    "axis": [0, 1, 0],
    "ang_min": 0,
    "ang_max": 0,
    "angdot": 0
}

rfin = {
    "type": "surface",
    "dims": [-0.247, 0.130],
    "coeff": 2,
    "origin": (np.array([0, -190, 0])*1e-3 - com).tolist(),
    "orient0": {"w": float(np.sqrt(2)/2), "x": float(np.sqrt(2)/2), "y": 0, "z": 0},
    "cenpres": [-0.247/2, 0.130/2, 0],
    "axis": [0, 1, 0],
    "ang_min": 0,
    "ang_max": 0,
    "angdot": 0
}

actuators = {
    "thruster": thruster,
    "lail": lail,
    "rail": rail,
    "lfin": lfin,
    "rfin": rfin
}

##################################################

from models import example_sensors
sensors = {}

sensors["accel"] = deepcopy(example_sensors.accelerometer)
sensors["accel"]["pos"] = (np.array([279.6, -25.3, -41.7])*1e-3 - com).tolist()
sensors["accel"]["rotmat"] = np.eye(3).tolist()

sensors["gyro"] = deepcopy(example_sensors.gyroscope)
sensors["gyro"]["pos"] = (np.array([279.6, -25.3, -41.7])*1e-3 - com).tolist()
sensors["gyro"]["rotmat"] = np.eye(3).tolist()

sensors["mag"] = deepcopy(example_sensors.magnetometer)
sensors["mag"]["pos"] = (np.array([279.6, -25.3, -41.7])*1e-3 - com).tolist()
sensors["mag"]["rotmat"] = np.eye(3).tolist()

sensors["baro"] = deepcopy(example_sensors.barometer)
sensors["baro"]["pos"] = (np.array([279.6, -25.3, -41.7])*1e-3 - com).tolist()

sensors["thermo"] = deepcopy(example_sensors.thermometer)
sensors["thermo"]["pos"] = (np.array([279.6, -25.3, -41.7])*1e-3 - com).tolist()

sensors["pitot"] = deepcopy(example_sensors.pitot)
sensors["pitot"]["pos"] = (np.array([128, 135, -26.6])*1e-3 - com).tolist()
sensors["pitot"]["dir"] = [1, 0, 0]

sensors["gps"] = deepcopy(example_sensors.gps)
sensors["gps"]["pos"] = (np.array([-24.8, -139.4, -26.6])*1e-3 - com).tolist()

##################################################

deltawing = {
    "actuators": actuators,
    "sensors": sensors,
    "body": body,
    "energy0": 1e99,
    "voltage0": 15,
    "power0": 18
}
