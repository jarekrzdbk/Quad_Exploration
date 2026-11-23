"""Workstation replay of recorded poses, including the tunnel's local voxel cloud."""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation

from Simulation import config
from Simulation.flight import ENU_FROM_NED


def animate(csv_path, flight):
    rows = np.genfromtxt(csv_path, delimiter=",", names=True, ndmin=1)
    positions = np.column_stack([rows[key] for key in ("east_m", "north_m", "up_m")])
    attitudes = Rotation.from_quat(np.column_stack([rows[key] for key in ("qx", "qy", "qz", "qw")]))
    cloud = getattr(flight.field, "pointcloud", np.empty((0, 3)))
    if config.orient == "NED" and cloud.size:
        cloud = ENU_FROM_NED.apply(cloud)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection="3d")
    ax.set(xlabel="East (m)", ylabel="North (m)", zlabel="Up (m)")
    ax.set_box_aspect((1, 1, 0.75))
    voxels, = ax.plot([], [], [], ".", color="0.6", markersize=2)
    track, = ax.plot([], [], [], color="teal", linewidth=1)
    arms = [ax.plot([], [], [], color=color, linewidth=3)[0] for color in ("darkorange", "navy")]
    dx, dy = flight.quad.params["dxm"], flight.quad.params["dym"]
    body = np.array([[-dx, -dy, 0], [dx, dy, 0], [-dx, dy, 0], [dx, -dy, 0]])

    def draw(index):
        pos = positions[index]
        if cloud.size:
            nearby = cloud[np.all(np.abs(cloud - pos) <= 3, axis=1)]
            voxels.set_data_3d(*nearby.T)
        track.set_data_3d(*positions[:index + 1].T)
        ends = attitudes[index].apply(body) + pos
        for line, pair in zip(arms, (ends[:2], ends[2:])):
            line.set_data_3d(*pair.T)
        for setter, centre, radius in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim),
                                          pos, (2.25, 2.25, 1.7)):
            setter(centre - radius, centre + radius)
        ax.set_title(f"Flight time {rows['time_s'][index]:.2f} s")

    # Display at 25 Hz, including the exact final sample for short recordings.
    frames = list(range(0, len(rows) - 1, 2)) + [len(rows) - 1]
    draw(0)
    animation = FuncAnimation(fig, draw, frames=frames, interval=40, repeat=False)
    return fig, animation
