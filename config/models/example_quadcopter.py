"""
Example configuration for a quadcopter craft.
Body initial conditions must still be set.
(See example_airplane for more detailed parameter comments).

"""
from __future__ import division
import numpy as np
from copy import deepcopy

##################################################

body = {
    "pos0": None,
    "ori0": None,
    "vel0": None,
    "avel0": None,
    "mass": 1,
    "inertia": [[0.01, 0.00, 0.00],
                [0.00, 0.01, 0.00],
                [0.00, 0.00, 0.02]],
    "centroid": [0, 0, 0],
    "bounding_box": [0.25, 0.25, 0.08],
    "areas": [0.05, 0.05, 0.1],
    "volume": 0.001,
    "cenpres": [0, 0, 0],
    "Cp1": np.diag([1, 1, 1]).tolist(),
    "Cp2": np.diag([0, 0, 0]).tolist(),
    "Cq": np.diag([1e-1, 1e-1, 1e-1]).tolist(),
    "bounciness": 0.2
}

##################################################

# Left-handed propeller, front-boom, pointed up
prop0 = {
    "type": "thruster",
    "dia": 0.125,
    "pos": [0.2, 0, 0],
    "dir": [0, 0, 1],
    "c1": 0,
    "c2": 1e-7,
    "kr": 0.05,
    "kv": 2e-6,
    "rpm_min": 0,
    "rpm_max": 10000,
    "deadband": 100,
    "rpmdot": 20000,
    "power_max": 140
}

# Right-handed propeller, left-boom, pointed up
prop1 = deepcopy(prop0)
prop1["pos"] = [0, 0.2, 0]
prop1["kr"] = -0.05

# Left-handed propeller, back-boom, pointed up
prop2 = deepcopy(prop0)
prop2["pos"] = [-0.2, 0, 0]

# Right-handed propeller, right-boom, pointed up
prop3 = deepcopy(prop1)
prop3["pos"] = [0, -0.2, 0]

actuators = {
    "prop0": prop0,
    "prop1": prop1,
    "prop2": prop2,
    "prop3": prop3,
}

##################################################

sensors = {}

##################################################

quadcopter = {
    "actuators": actuators,
    "sensors": sensors,
    "body": body,
    "energy0": 1e99,
    "voltage0": 15,
    "power0": 18
}
