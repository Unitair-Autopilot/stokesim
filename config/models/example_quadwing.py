"""
Baseline example model for a quad-winged aircraft.
Body initial conditions must still be set.
(See example_airplane for more detailed parameter comments).

"""
from __future__ import division
import numpy as np
from copy import deepcopy

##################################################

com = np.array([-190, 0.0, 0.0])*1e-3

body = {
    "pos0": None,
    "ori0": None,
    "vel0": None,
    "avel0": None,
    "mass": 1.237,  # 1.237 light, 2 medium, 2.5 heavy
    "inertia": (np.array([[32819303.16,    53326.85,  1916061.52],
                          [   53326.85, 31715174.29,   -21723.38],
                          [ 1916061.52,   -21723.38, 62787272.05]])*1e-9/2*1.237).tolist(),
    "centroid": (np.array([-189, 0, 0])*1e-3 - com).tolist(),
    "bounding_box": [0.38, 0.64, 0.13],
    "areas": (np.array([11.145, 26.0, 165.25])*0.00064516).tolist(),
    "volume": 0.002,
    "cenpres": (np.array([-189, 0, 0])*1e-3 - com).tolist(),
    "Cp1": np.diag([0.5, 0.5, 0.5]).tolist(),
    "Cp2": [[ 0.5, 0.0, -0.1],
            [ 0.0, 1.0,  0.0],
            [-0.1, 0.0,  4.0]],
    "Cq": np.diag([0.02, 0.02, 0.02]).tolist(),
    "bounciness": 0.001
}

##################################################

thruster = {
    "type": "thruster",
    "dia": 0.25,
    "pos": (np.array([-15.5, 0, 0.0276])*0.0254 - com).tolist(),
    "dir": [1, 0, 0],
    "c1": 1.7092e-04,
    "c2": 4.5811e-08 * 0.375,
    "kr": -0.0096 * 0.6,
    "kv": 5e-4 * 0.01,
    "ks": 0.0,
    "rpm_min": 0,
    "rpm_max": 16000,
    "deadband": 800,
    "rpmdot": 30000,
    "power_max": 200
}

lailf = {
    "type": "surface",
    "dims": (np.array([-0.625, 7.75])*0.0254).tolist(),
    "coeff": 3.0,
    "origin": (np.array([-3.175, 13-3.25-7.75, 0.95714])*0.0254 - com).tolist(),
    "orient0": {"w": 1, "x": 0, "y": 0, "z": 0},
    "cenpres": (np.array([-0.625, 7.75, 0])*0.0254/2).tolist(),
    "axis": [0, -1, 0],
    "ang_min": -20,
    "ang_max": 20,
    "angdot": 270
}

railf = {
    "type": "surface",
    "dims": (np.array([-0.625, 7.75])*0.0254).tolist(),
    "coeff": 3.0,
    "origin": (np.array([-3.175, -13+3.25, 0.95714])*0.0254 - com).tolist(),
    "orient0": {"w": 1, "x": 0, "y": 0, "z": 0},
    "cenpres": (np.array([-0.625, 7.75, 0])*0.0254/2).tolist(),
    "axis": [0, 1, 0],
    "ang_min": -20,
    "ang_max": 20,
    "angdot": 270
}

lailb = {
    "type": "surface",
    "dims": (np.array([-0.625, 7.75])*0.0254).tolist(),
    "coeff": 3.0,
    "origin": (np.array([-13.375, 12.5-3.25-7.75, -1.045])*0.0254 - com).tolist(),
    "orient0": {"w": 1, "x": 0, "y": 0, "z": 0},
    "cenpres": (np.array([-0.625, 7.75, 0])*0.0254/2).tolist(),
    "axis": [0, -1, 0],
    "ang_min": -20,
    "ang_max": 20,
    "angdot": 270
}

railb = {
    "type": "surface",
    "dims": (np.array([-0.625, 7.75])*0.0254).tolist(),
    "coeff": 3.0,
    "origin": (np.array([-13.375, -12.5+3.25, -1.045])*0.0254 - com).tolist(),
    "orient0": {"w": 1, "x": 0, "y": 0, "z": 0},
    "cenpres": (np.array([-0.625, 7.75, 0])*0.0254/2).tolist(),
    "axis": [0, 1, 0],
    "ang_min": -20,
    "ang_max": 20,
    "angdot": 270
}

lfin = {
    "type": "surface",
    "dims": (np.array([-2, 1.625])*0.0254).tolist(),
    "coeff": 4.0,
    "origin": (np.array([-12, 13.45, -1.045])*0.0254 - com).tolist(),
    "orient0": {"w": float(np.sqrt(2)/2), "x": float(np.sqrt(2)/2), "y": 0, "z": 0},
    "cenpres": (np.array([-2, 1.625, 0])*0.0254/2).tolist(),
    "axis": [0, 1, 0],
    "ang_min": 0,
    "ang_max": 0,
    "angdot": 0
}

rfin = {
    "type": "surface",
    "dims": (np.array([-2, 1.625])*0.0254).tolist(),
    "coeff": 4.0,
    "origin": (np.array([-12, -13.45, -1.045])*0.0254 - com).tolist(),
    "orient0": {"w": float(np.sqrt(2)/2), "x": float(np.sqrt(2)/2), "y": 0, "z": 0},
    "cenpres": (np.array([-2, 1.625, 0])*0.0254/2).tolist(),
    "axis": [0, 1, 0],
    "ang_min": 0,
    "ang_max": 0,
    "angdot": 0
}

actuators = {
    "thruster": thruster,
    "lailf": lailf,
    "railf": railf,
    "lailb": lailb,
    "railb": railb,
    "lfin": lfin,
    "rfin": rfin
}

##################################################

from models import example_sensors
sensors = {}

sensors["accel"] = deepcopy(example_sensors.accelerometer)
sensors["accel"]["pos"] = [0, 0, 0]
sensors["accel"]["rotmat"] = np.eye(3).tolist()

sensors["gyro"] = deepcopy(example_sensors.gyroscope)
sensors["gyro"]["pos"] = [0, 0, 0]
sensors["gyro"]["rotmat"] = np.eye(3).tolist()

sensors["mag"] = deepcopy(example_sensors.magnetometer)
sensors["mag"]["pos"] = [0, 0, 0]
sensors["mag"]["rotmat"] = np.eye(3).tolist()

sensors["baro"] = deepcopy(example_sensors.barometer)
sensors["baro"]["pos"] = [0, 0, 0]

sensors["thermo"] = deepcopy(example_sensors.thermometer)
sensors["thermo"]["pos"] = [0, 0, 0]

sensors["pitot"] = deepcopy(example_sensors.pitot)
sensors["pitot"]["pos"] = [0, 0.0625, 0]
sensors["pitot"]["dir"] = [1, 0, 0]

sensors["gps"] = deepcopy(example_sensors.gps)
sensors["gps"]["pos"] = [0, 0, 0]

##################################################

quadwing = {
    "actuators": actuators,
    "sensors": sensors,
    "body": body,
    "energy0": 1e99,
    "voltage0": 15,
    "power0": 18
}
