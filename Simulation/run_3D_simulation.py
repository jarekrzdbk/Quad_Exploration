"""Simulate the original tunnel flight, then replay it in a VisPy desktop window.

Original model and visualization by John Bass, john.bobzwik@gmail.com, MIT.
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Simulation.flight import DT, Flight


@dataclass
class Recording:
    flight: Flight
    times: np.ndarray
    positions: np.ndarray
    quaternions: np.ndarray
    active_voxels: list
    diagnostics: list


def record(ticks, diagnostic_plots):
    flight = Flight("tunnel")
    times, positions, quaternions, active, history = [], [], [], [], []
    start = time.perf_counter()
    for tick in range(ticks + 1):
        if tick:
            flight.step()
        if tick % 4 == 0 or tick == ticks:
            p, q = flight.pose()
            times.append(flight.time)
            positions.append(p)
            quaternions.append(q)
            active.append(flight.field.idx_withinField.copy())
        if diagnostic_plots:
            quad = flight.quad
            values = (flight.time, quad.pos, quad.vel, quad.quat, quad.omega, quad.euler,
                      flight.ctrl.w_cmd, quad.wMotor, quad.thr, quad.tor, flight.traj.sDes,
                      flight.ctrl.sDesCalc, flight.field.distanceMin)
            history.append(tuple(np.copy(v) for v in values))
        if tick and tick % 1000 == 0:
            print(f"Simulated {flight.time:g} / {ticks * DT:g} s", flush=True)
    print(f"Simulated {flight.time:.3f} s in {time.perf_counter() - start:.2f} s. Opening replay.", flush=True)
    return Recording(flight, np.asarray(times), np.asarray(positions), np.asarray(quaternions),
                     active, history)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=95.0, help="duration in model seconds")
    parser.add_argument("--renderer", choices=("boxes", "markers"), default="boxes")
    parser.add_argument("--no-plots", action="store_true", help="skip diagnostic figures after closing the 3D window")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < DT:
        parser.error(f"--seconds must be finite and at least {DT}")
    ticks = round(args.seconds / DT)
    if not math.isclose(ticks * DT, args.seconds, abs_tol=1e-8, rel_tol=0):
        parser.error(f"--seconds must be a multiple of {DT}")
    try:
        from Simulation.vispy_replay import show
    except ModuleNotFoundError as err:
        if err.name not in ("vispy", "glfw"):
            raise
        parser.exit(1, "Install the desktop dependencies with ./install_local.sh --vispy\n")
    recording = record(ticks, not args.no_plots)
    show(recording, args.renderer)
    if recording.diagnostics:
        from matplotlib import pyplot as plt
        from Simulation.utils.display import makeFigures
        *series, distance = (np.asarray(v) for v in zip(*recording.diagnostics))
        makeFigures(recording.flight.quad.params, *series, recording.flight.field, distance)
        plt.show()


if __name__ == "__main__":
    main()
