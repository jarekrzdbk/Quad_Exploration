"""Run the Python model on a workstation or a headless machine."""

import argparse
import csv
import math
from pathlib import Path

from flight import CSV_HEADER, DT, Flight


def plot_route(csv_path, plot_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    rows = np.genfromtxt(csv_path, names=True, delimiter=",", ndmin=1)
    fig, (plan, altitude) = plt.subplots(1, 2, figsize=(9, 4), constrained_layout=True)
    plan.plot(rows["east_m"], rows["north_m"])
    plan.scatter(rows["east_m"][-1], rows["north_m"][-1], label="Final position")
    plan.set(xlabel="East (m)", ylabel="North (m)", aspect="equal")
    plan.legend()
    altitude.plot(rows["time_s"], rows["up_m"])
    altitude.set(xlabel="Time (s)", ylabel="Height above model origin (m)")
    for axis in (plan, altitude):
        axis.grid(alpha=0.3)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("room", "tunnel"), default="tunnel")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--output", type=Path, default=Path("build/flight.csv"))
    parser.add_argument("--plot", type=Path, help="also save a plan view and altitude plot (PNG)")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < DT:
        parser.error(f"--seconds must be finite and at least {DT}")
    ticks = round(args.seconds / DT)
    if not math.isclose(ticks * DT, args.seconds, rel_tol=0, abs_tol=1e-8):
        parser.error(f"--seconds must be a multiple of the {DT} s model tick")
    if args.plot and args.output.resolve() == args.plot.resolve():
        parser.error("--output and --plot need different file paths")
    flight = Flight(args.scenario)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_HEADER)
        for tick in range(ticks + 1):
            if tick:
                flight.step()
            # Sample at 50 Hz and always include the exact final tick.
            if tick % 4 == 0 or tick == ticks:
                position, attitude = flight.pose()
                writer.writerow((f"{flight.time:.3f}", *position, *attitude))
    print(f"{args.scenario}: {flight.time:.3f} s, final ENU (m) {position}, CSV {args.output}")
    if args.plot:
        plot_route(args.output, args.plot)
        print(f"Route plot: {args.plot}")


if __name__ == "__main__":
    main()
