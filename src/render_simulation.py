"""Render the hybrid hopping controller to an MP4 (headless, cross-platform).

Works on machines where the interactive GLFW viewer crashes (e.g. some Intel
Xe / hybrid-GPU Linux drivers), because it renders offscreen.

Usage:
    python src/render_simulation.py [out.mp4] [seconds]
"""
import sys
from pathlib import Path

from render_utils import select_gl_backend, write_video

select_gl_backend()

import mujoco  # noqa: E402  (must come after select_gl_backend)
import numpy as np  # noqa: E402

import hybrid_controller_test as hc  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"
WIDTH, HEIGHT, FPS = 640, 480, 30


def make_camera():
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.25, 0.0, 0.35]
    cam.distance = 1.6
    cam.azimuth = 90
    cam.elevation = -15
    return cam


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "renders" / "hopping.mp4"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    hc.set_joint(model, data, "gantry_pitch", 0.0)
    hc.set_joint(model, data, "hip", 0.35)
    hc.set_joint(model, data, "knee", -0.70)
    hc.set_joint_velocity(model, data, "gantry_pitch", 0.50)
    mujoco.mj_forward(model, data)

    state = hc.STANCE if hc.foot_in_contact(model, data) else hc.FLIGHT
    state_start = data.time
    stance_start = data.time if state == hc.STANCE else None

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = make_camera()

    steps_per_frame = max(1, round((1.0 / FPS) / model.opt.timestep))
    total_steps = int(seconds / model.opt.timestep)
    frames = []

    for step in range(total_steps):
        contact = hc.foot_in_contact(model, data)
        t_in_state = data.time - state_start

        if state == hc.FLIGHT and contact and t_in_state >= hc.MIN_FLIGHT_TIME:
            state, state_start, stance_start = hc.STANCE, data.time, data.time
        elif state == hc.STANCE and (
            ((not contact) and t_in_state >= hc.MIN_STANCE_TIME)
            or t_in_state >= hc.MAX_STANCE_TIME
        ):
            state, state_start, stance_start = hc.FLIGHT, data.time, None

        if state == hc.FLIGHT:
            tau = hc.flight_control(model, data)
        else:
            tau = hc.stance_control(model, data, data.time - stance_start)

        tau = np.clip(tau, -hc.TORQUE_LIMIT, hc.TORQUE_LIMIT)
        data.ctrl[0], data.ctrl[1] = tau[0], tau[1]
        mujoco.mj_step(model, data)

        if step % steps_per_frame == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    renderer.close()
    out = write_video(frames, out_path, fps=FPS)
    print(f"Wrote {len(frames)} frames ({len(frames) / FPS:.1f}s) -> {out}")


if __name__ == "__main__":
    main()
