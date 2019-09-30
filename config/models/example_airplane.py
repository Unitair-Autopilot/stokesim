"""
Example configuration for an airplane craft.
Body initial conditions must still be set.

"""
from __future__ import division
import numpy as np
from copy import deepcopy

##################################################

body = {
    "pos0": None,  # initial position (ENU)
    "ori0": None,  # initial orientation quaternion (body in ENU)
    "vel0": None,  # initial velocity (body)
    "avel0": None,  # initial angular velocity (body)
    "mass": 2,  # total mass
    "inertia": [[0.100, 0.000, 0.001],
                [0.000, 0.100, 0.000],
                [0.001, 0.000, 1.000]],  # inertia tensor (body)
    "centroid": [0, 0, 0],  # geometric center (body)
    "bounding_box": [1, 1, 0.3],  # minimum bounding-box lengths along body axes centered on centroid
    "areas": [0.025, 0.1, 0.5],  # maximum cross-sectional areas along body axes
    "volume": 0.05,  # total volume
    "cenpres": [0, 0, 0],  # center of pressure (body)
    "Cp1": np.diag([7, 18, 35]).tolist(),  # translational linear-drag tensor (body)
    "Cp2": np.diag([3, 3, 3]).tolist(),  # translational quadratic-drag tensor (body)
    "Cq": np.diag([0.1, 0.1, 1]).tolist(),  # rotational linear-drag tensor (body)
    "bounciness": 0.05  # coefficient of... "bounce" for collisions, 0 to 1
}

##################################################

from models import example_actuators
actuators = {}

actuators["lprop"] = deepcopy(example_actuators.thruster)
actuators["lprop"]["pos"] = [0, 0.21875, 0]
actuators["lprop"]["dir"] = [1, 0, 0]

actuators["rprop"] = deepcopy(example_actuators.thruster)
actuators["rprop"]["pos"] = [0, -0.21875, 0]
actuators["rprop"]["dir"] = [1, 0, 0]
actuators["rprop"]["kr"] = -actuators["lprop"]["kr"]

actuators["lail"] = deepcopy(example_actuators.surface)
actuators["lail"]["origin"] = [0, 0.0625, 0]
actuators["lail"]["orient0"] = {"w": 1, "x": 0, "y": 0, "z": 0}

actuators["rail"] = deepcopy(example_actuators.surface)
actuators["rail"]["origin"] = [0, -0.5, 0]
actuators["rail"]["orient0"] = {"w": 1, "x": 0, "y": 0, "z": 0}

actuators["elv"] = deepcopy(example_actuators.surface)
actuators["elv"]["dims"] = [-0.125, 0.75]
actuators["elv"]["coeff"] = 3
actuators["elv"]["origin"] = [-0.5, -0.375, 0.2]
actuators["elv"]["orient0"] = {"w": 1, "x": 0, "y": 0, "z": 0}
actuators["elv"]["cenpres"] = [-0.0625, 0.375, 0]

actuators["rud"] = deepcopy(example_actuators.surface)
actuators["rud"]["dims"] = [-0.125, 0.2]
actuators["rud"]["coeff"] = 3
actuators["rud"]["origin"] = [-0.375, 0, 0]
actuators["rud"]["orient0"] = {"w": float(np.sqrt(2)/2), "x": float(np.sqrt(2)/2), "y": 0, "z": 0}
actuators["rud"]["cenpres"] = [-0.0625, 0.1, 0]

##################################################

sensors = {}

##################################################

airplane = {
    "actuators": actuators,
    "sensors": sensors,
    "body": body,
    "energy0": 1e99,  # total battery energy, J
    "voltage0": 15,  # nominal battery voltage, V
    "power0": 18  # power consumed by electronics excluding actuators, W
}
