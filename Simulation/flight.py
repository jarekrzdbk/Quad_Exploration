"""Headless entry point for the upstream quadcopter, controller and trajectories."""

from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

import config
from ctrl import Control
from potentialField import PotField
from quadFiles.quad import Quadcopter
from trajectory import Trajectory
from utils.windModel import Wind


DT = 0.005
ENU_FROM_NED = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
FRD_FROM_FLU = Rotation.from_matrix(np.diag([1, -1, -1]))
CSV_HEADER = ("time_s", "east_m", "north_m", "up_m", "qw", "qx", "qy", "qz")


def update_commands(t, dt, quad, ctrl, traj, field):
    traj.desiredState(t, dt, quad)
    if field.pfVel or field.pfSatFor or field.pfFor:
        field.isWithinRange(quad)
        field.isWithinField(quad)
        field.rep_force(quad, traj)
    ctrl.controller(traj, quad, field, dt)


def quad_sim(t, dt, quad, ctrl, wind, traj, field):
    quad.update(t, dt, ctrl.w_cmd, wind)
    update_commands(t + dt, dt, quad, ctrl, traj, field)
    return t + dt


class Flight:
    """Advance one 5 ms model tick per step. pose() returns ENU metres, FLU wxyz."""

    def __init__(self, scenario="tunnel"):
        if config.orient not in ("NED", "ENU"):
            raise ValueError(f"Unknown model frame {config.orient!r}")
        self.ticks = 0
        self.quad = Quadcopter(0.0)
        if scenario == "tunnel":
            self.traj = Trajectory(self.quad, "xyz_pos", [2, 4, 0])
            self.field = PotField(1)
        elif scenario == "room":
            self.traj = Trajectory(self.quad, "xyz_pos", [1, 1, 0])
            self.traj.t_wps = np.array([0.0, 2.5, 5.0])
            waypoints_enu = np.array([[0, 0, 0.9], [1.2, 0, 0.9], [1.2, 1.2, 0.9]])
            self.traj.wps = (ENU_FROM_NED.inv().apply(waypoints_enu)
                             if config.orient == "NED" else waypoints_enu)
            self.traj.y_wps = np.zeros(3)
            self.field = SimpleNamespace(pfVel=0, pfFor=0, pfSatFor=0, F_rep=np.zeros(3))
        else:
            raise ValueError(f"Unknown scenario {scenario!r}")
        self.ctrl = Control(self.quad, self.traj.yawType)
        self.wind = Wind("None", 0.0, 0.0, 0.0)
        update_commands(0.0, DT, self.quad, self.ctrl, self.traj, self.field)

    @property
    def time(self):
        return self.ticks * DT

    def step(self):
        quad_sim(self.time, DT, self.quad, self.ctrl, self.wind, self.traj, self.field)
        self.ticks += 1

    def pose(self):
        position = self.quad.pos
        w, x, y, z = self.quad.quat
        attitude = Rotation.from_quat([x, y, z, w])
        if config.orient == "NED":
            position = ENU_FROM_NED.apply(position)
            attitude = ENU_FROM_NED * attitude * FRD_FROM_FLU
        x, y, z, w = attitude.as_quat()
        return tuple(position), (w, x, y, z)
