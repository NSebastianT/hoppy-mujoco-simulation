"""Hybrid hopping controller for the official CAD physics model.

Runs the FLIGHT/STANCE controller on models/hoppy_cad_physics.xml (the official
URDF assembly with real physics) and renders the robot hopping. The actuated
joints are joint3 (hip) and joint4 (knee); joint1 (yaw) and joint2 (pitch) are
passive. Headless, cross-platform.

Usage:
    python src/cad_hop_controller.py [out.mp4] [seconds]
"""
import sys
from pathlib import Path

from render_utils import select_gl_backend, write_video

select_gl_backend()

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "hoppy_cad_physics.xml"
WIDTH, HEIGHT, FPS = 640, 480, 30

TORQUE_LIMIT = np.array([12.2, 13.0])
MIN_STANCE, MAX_STANCE, MIN_FLIGHT = 0.10, 0.28, 0.16
PUSH = 300.0
RAMP_TIME = 0.12
FLIGHT_REF = (0.30, -0.50)   # joint3, joint4 tucked for landing
STANCE_REF = (0.0, 0.0)      # extended for push-off


class Hopper:
    def __init__(self, model):
        self.m = model
        self.qa = {n: model.jnt_qposadr[self._jid(n)] for n in ("joint3", "joint4")}
        self.va = {n: model.jnt_dofadr[self._jid(n)] for n in ("joint3", "joint4")}
        self.foot_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot")
        self.floor_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")

    def _jid(self, name):
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)

    def in_contact(self, d):
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {self.foot_g, self.floor_g}:
                return True
        return False

    def foot_jac(self, d):
        jp = np.zeros((3, self.m.nv))
        jr = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, d, jp, jr, self.foot_s)
        return jp[:, [self.va["joint3"], self.va["joint4"]]]

    def _pd(self, d, r3, r4, kp3, kd3, kp4, kd4):
        q3, q4 = d.qpos[self.qa["joint3"]], d.qpos[self.qa["joint4"]]
        v3, v4 = d.qvel[self.va["joint3"]], d.qvel[self.va["joint4"]]
        return np.array([kp3 * (r3 - q3) - kd3 * v3, kp4 * (r4 - q4) - kd4 * v4])

    def flight(self, d):
        return self._pd(d, *FLIGHT_REF, 6.0, 1.0, 4.5, 1.0)

    def stance(self, d, stance_time):
        r = np.clip(stance_time / RAMP_TIME, 0.0, 1.0)
        r = 3.0 * r ** 2 - 2.0 * r ** 3
        tau = self.foot_jac(d).T @ np.array([0.0, 0.0, -PUSH * r])
        tau += self._pd(d, *STANCE_REF, 10.0, 1.5, 8.0, 1.5)
        return tau


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "renders" / "cad_hopping.mp4"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    h = Hopper(model)
    state = "STANCE" if h.in_contact(data) else "FLIGHT"
    t_state = data.time
    t_stance = data.time if state == "STANCE" else None

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [-0.35, 0.0, 0.25]
    cam.distance = 1.9
    cam.azimuth = 110
    cam.elevation = -12

    steps_per_frame = max(1, round((1.0 / FPS) / model.opt.timestep))
    frames = []
    hops = 0

    for step in range(int(seconds / model.opt.timestep)):
        contact = h.in_contact(data)
        t_in = data.time - t_state
        if state == "FLIGHT" and contact and t_in >= MIN_FLIGHT:
            state, t_state, t_stance = "STANCE", data.time, data.time
        elif state == "STANCE" and ((not contact and t_in >= MIN_STANCE) or t_in >= MAX_STANCE):
            state, t_state, t_stance = "FLIGHT", data.time, None
            hops += 1

        tau = h.flight(data) if state == "FLIGHT" else h.stance(data, data.time - t_stance)
        data.ctrl[:] = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
        mujoco.mj_step(model, data)

        if step % steps_per_frame == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    renderer.close()
    out = write_video(frames, out_path, fps=FPS)
    print(f"Wrote {len(frames)} frames, {hops} hops -> {out}")


if __name__ == "__main__":
    main()
