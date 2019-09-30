"""
Example configurations for the actuator types.
Positions and orientations must still be set.

"""
from __future__ import division
import numpy as np

##################################################

thruster = {
    "type": "thruster",
    "dia": 0.125,  # diameter of the propeller, m
    "pos": None,  # position vector in COM-coordinates of mounting point, m
    "dir": None,  # direction vector in COM-coordinates of thrust at positive RPM
    "c1": 2e-6,  # linear coefficient for thrust from RPM
    "c2": 2e-6,  # quadratic coefficient for thrust from RPM
    "kr": 0.05,  # ratio of reaction torque to thrust (sign defines handedness)
    "kv": 2e-6,  # thrust loss per unit squared velocity (>=0)
    "ks": 0.0,  # linear coefficient for slipstream wind from thrust, (m/s)/N
    "rpm_min": 0,  # minimum RPM
    "rpm_max": 2100,  # maximum RPM
    "deadband": 25,  # plus-or-minus RPM command dead-band (>=0)
    "rpmdot": 10000,  # angular acceleration magnitude (>0)
    "power_max": -1  # power consumption at maximum RPM (negative implies use reaction_torque*RPM), W
}

##################################################

surface = {
    "type": "surface",
    "dims": [-0.125, 0.4375],  # lengths along the surface 0 and 1 directions, m
    "coeff": 2,  # coefficient for quadratic-drag inducing face, (>0)
    "origin": None,  # mounting point and origin of surface-coordinates in COM-coordinates, m
    "orient0": None,  # trim orientation of surface-coordinates relative to COM-coordinates
    "cenpres": [-0.0625, 0.21875, 0],  # vector in surface-coordinates of the center of pressure, m
    "axis": [0, 1, 0],  # direction vector in surface-coordinates about which the surface can be rotated
    "ang_min": -10,  # minimum deflection angle, deg
    "ang_max": 10,  # maximum deflection angle, deg
    "angdot": 100  # angular velocity magnitude for rotating the surface, deg/s (>0)
}
