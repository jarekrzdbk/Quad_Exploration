"""OpenGL playback of the tunnel recording in the exporter's ENU world."""
import time

import glfw  # Check the optional backend dependency before computing a long flight.
import numpy as np
from scipy.spatial.transform import Rotation
from vispy import app, scene
from vispy.geometry import create_box

from Simulation import config
from Simulation.flight import ENU_FROM_NED


def show(recording, renderer):
    app.use_app("glfw")
    canvas = scene.SceneCanvas(keys="interactive", size=(960, 640), bgcolor="#151c26")
    view = canvas.central_widget.add_view()
    view.camera = scene.cameras.TurntableCamera(distance=3.6, elevation=16, azimuth=25, fov=65)
    flight = recording.flight
    cloud = flight.field.pointcloud
    waypoints = flight.traj.wps
    if config.orient == "NED":
        cloud = ENU_FROM_NED.apply(cloud)
        waypoints = ENU_FROM_NED.apply(waypoints)
    cloud = np.asarray(cloud, dtype=np.float32)
    unit, faces, _ = create_box()
    yellow, red = (0.9, 0.8, 0.36, 1), (0.95, 0.19, 0.13, 1)

    def voxel_visual(color):
        if renderer == "markers":
            return scene.visuals.Markers(parent=view.scene)
        mesh = scene.visuals.Mesh(color=color, shading="flat", parent=view.scene)
        mesh.set_gl_state("opaque", depth_test=True, cull_face=False)
        return mesh

    def set_voxels(visual, points, size, color, pixels):
        visual.visible = bool(len(points))
        if not visual.visible:
            return
        if renderer == "markers":
            visual.set_data(pos=points, face_color=color, edge_width=0, size=pixels)
        else:
            vertices = points[:, None, :] + size * unit["position"]
            offsets = (np.arange(len(points), dtype=np.uint32) * len(unit))[:, None, None]
            visual.set_data(vertices=vertices.reshape(-1, 3), faces=(faces + offsets).reshape(-1, 3))

    obstacles, highlighted = voxel_visual(yellow), voxel_visual(red)
    set_voxels(obstacles, cloud, 0.055, yellow, 4)
    goals = scene.visuals.Markers(parent=view.scene)
    goals.set_data(pos=waypoints.astype(np.float32), face_color="lime", size=10, edge_width=0)
    arm_lines = [scene.visuals.Line(color=c, width=4, parent=view.scene) for c in ("red", "deepskyblue")]
    track = scene.visuals.Line(color="tomato", width=2, parent=view.scene)
    dx, dy = flight.quad.params["dxm"], flight.quad.params["dym"]
    arms = np.array([[-dx, -dy, 0], [dx, dy, 0], [-dx, dy, 0], [dx, -dy, 0]])
    attitudes = Rotation.from_quat(recording.quaternions[:, [1, 2, 3, 0]])
    start = None
    frame = -1

    def draw(index):
        pos = recording.positions[index]
        ends = (attitudes[index].apply(arms) + pos).astype(np.float32)
        for line, pair in zip(arm_lines, (ends[:2], ends[2:])):
            line.set_data(pos=pair)
        track.set_data(pos=recording.positions[:index + 1].astype(np.float32))
        set_voxels(highlighted, cloud[recording.active_voxels[index]], 0.105, red, 7)
        view.camera.center = tuple(pos)
        canvas.title = f"Tunnel • {renderer} • {recording.times[index]:.2f} / {recording.times[-1]:.2f} s"

    def update(event):
        nonlocal frame
        elapsed = time.perf_counter() - start
        i = max(0, recording.times.searchsorted(elapsed, side="right") - 1)
        if i != frame:
            draw(i)
            frame = i
        if i == len(recording.times) - 1:
            timer.stop()

    timer = app.Timer(1 / 50, connect=update)

    @canvas.events.draw.connect
    def begin(event):
        nonlocal start
        if start is None:
            start = time.perf_counter()
            timer.start()

    canvas.events.close.connect(lambda event: timer.stop())
    draw(0)
    canvas.show()
    app.run()
