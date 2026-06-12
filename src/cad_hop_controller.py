"""Hybrid hopping controller for the CAD physics model."""
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
MIN_STANCE = 0.12
MAX_STANCE = 0.22
MIN_FLIGHT = 0.10
PUSH_TIME = 0.15
BLEND_TIME = 0.010
PUSH_PEAK = 560.0
HORIZONTAL_FORCE = 35.0
WARMUP_TIME = 1.0
WARMUP_START = 0.00
SOFT_ENGAGE_TIME = 0.50
ENCODER_COUNTS_PER_REV = 751.8
ENCODER_STEP = 2.0 * np.pi / ENCODER_COUNTS_PER_REV
VELOCITY_FILTER_LAMBDA = 15.0

FLIGHT_REF = np.array([0.26, -0.48])
STANCE_REF = np.array([0.08, 0.0])
FLIGHT_KP_CART = 200.0
FLIGHT_POSTURE_KP = np.array([8.0, 7.0])
FLIGHT_KD = np.array([1.3, 1.2])
STANCE_KP = np.array([72.0, 64.0])
STANCE_KD = np.array([4.2, 3.8])


class Hopper:
    def __init__(self, model, push_peak=PUSH_PEAK, horizontal_force=HORIZONTAL_FORCE):
        self.m = model
        self.push_peak = PUSH_PEAK if push_peak is None else push_peak
        self.horizontal_force = HORIZONTAL_FORCE if horizontal_force is None else horizontal_force
        self.qa = {n: model.jnt_qposadr[self._jid(n)] for n in ("joint3", "joint4")}
        self.va = {n: model.jnt_dofadr[self._jid(n)] for n in ("joint3", "joint4")}
        self.foot_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot")
        self.floor_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.lower_leg_g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "lower_leg_collision")
        self.foot_s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
        self.state = "FLIGHT"
        self.state_start = 0.0
        self.stance_start = 0.0
        self.flight_start = 0.0
        self.air_start = None
        self.air_times = []
        self.liftoffs = 0
        self.q_meas = np.zeros(2)
        self.v_est = np.zeros(2)
        self.prev_q_meas = None
        self._fk = mujoco.MjData(model)

    def _jid(self, name):
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)

    def reset(self, data):
        contact = self.in_contact(data)
        self.state = "STANCE" if contact else "FLIGHT"
        self.state_start = data.time
        self.stance_start = data.time if contact else 0.0
        self.flight_start = data.time if not contact else 0.0
        self.air_start = None if contact else data.time
        self.air_times.clear()
        self.liftoffs = 0
        self.q_meas = self.quantized_actuated_positions(data)
        self.prev_q_meas = self.q_meas.copy()
        self.v_est[:] = 0.0

    def quantized_actuated_positions(self, data):
        q = np.array([data.qpos[self.qa["joint3"]], data.qpos[self.qa["joint4"]]])
        return np.round(q / ENCODER_STEP) * ENCODER_STEP

    def update_encoder(self, data):
        q_now = self.quantized_actuated_positions(data)
        if self.prev_q_meas is None:
            self.prev_q_meas = q_now.copy()
            self.q_meas = q_now
            return
        dt = self.m.opt.timestep
        raw_velocity = (q_now - self.prev_q_meas) / dt
        alpha = (VELOCITY_FILTER_LAMBDA * dt) / (1.0 + VELOCITY_FILTER_LAMBDA * dt)
        self.v_est += alpha * (raw_velocity - self.v_est)
        self.prev_q_meas = q_now.copy()
        self.q_meas = q_now

    def in_contact(self, data):
        for i in range(data.ncon):
            c = data.contact[i]
            pair = {c.geom1, c.geom2}
            if pair == {self.foot_g, self.floor_g} or pair == {self.lower_leg_g, self.floor_g}:
                return True
        return False

    def foot_jac(self, data):
        jp = np.zeros((3, self.m.nv))
        jr = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, data, jp, jr, self.foot_s)
        return jp[:, [self.va["joint3"], self.va["joint4"]]]

    def _pd(self, data, ref, kp, kd):
        q = np.array([data.qpos[self.qa["joint3"]], data.qpos[self.qa["joint4"]]])
        return kp * (ref - q) - kd * self.v_est

    @staticmethod
    def _smooth01(x):
        x = np.clip(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    @staticmethod
    def _bezier_pulse(x):
        x = np.clip(x, 0.0, 1.0)
        om = 1.0 - x
        return 3.0 * om * om * x * 1.35 + 3.0 * om * x * x * 1.35

    def flight_target(self, data):
        # foot position the reference pose would give at the current yaw/pitch
        self._fk.qpos[:] = data.qpos
        self._fk.qpos[self.qa["joint3"]] = FLIGHT_REF[0]
        self._fk.qpos[self.qa["joint4"]] = FLIGHT_REF[1]
        mujoco.mj_kinematics(self.m, self._fk)
        return self._fk.site_xpos[self.foot_s].copy()

    def flight(self, data):
        # cartesian PD on the foot via the transposed Jacobian, plus a weak
        # joint posture term: hip and knee move the foot in nearly the same
        # direction here, so J^T alone leaves one joint direction unstiffened
        q = np.array([data.qpos[self.qa["joint3"]], data.qpos[self.qa["joint4"]]])
        err = self.flight_target(data) - data.site_xpos[self.foot_s]
        tau = self.foot_jac(data).T @ (FLIGHT_KP_CART * err)
        return tau + FLIGHT_POSTURE_KP * (FLIGHT_REF - q) - FLIGHT_KD * self.v_est

    def stance(self, data, stance_time):
        phase = stance_time / PUSH_TIME
        warmup = WARMUP_START + (1.0 - WARMUP_START) * self._smooth01(data.time / WARMUP_TIME)
        force = self.push_peak * self._bezier_pulse(phase)
        alpha = self._smooth01(stance_time / BLEND_TIME)
        foot_xy = data.site_xpos[self.foot_s, :2]
        radial_norm = np.linalg.norm(foot_xy)
        tangent = np.zeros(3)
        if radial_norm > 1e-6:
            radial = foot_xy / radial_norm
            tangent[:2] = [-radial[1], radial[0]]
        # the warmup only ramps the Bezier push; the posture PD runs at full
        # strength from the start so the leg never gets cranked while weak
        force_world = warmup * np.array([0.0, 0.0, -force]) + self.horizontal_force * tangent
        force_tau = self.foot_jac(data).T @ force_world
        stance_tau = force_tau + self._pd(data, STANCE_REF, STANCE_KP, STANCE_KD)
        return (1.0 - alpha) * self.flight(data) + alpha * stance_tau

    def update_state(self, data):
        contact = self.in_contact(data)
        elapsed = data.time - self.state_start
        if self.state == "FLIGHT":
            if contact and elapsed >= MIN_FLIGHT:
                if self.air_start is not None:
                    self.air_times.append(data.time - self.air_start)
                self.state = "STANCE"
                self.state_start = data.time
                self.stance_start = data.time
                self.air_start = None
        elif (not contact and elapsed >= MIN_STANCE) or elapsed >= MAX_STANCE:
            self.state = "FLIGHT"
            self.state_start = data.time
            self.flight_start = data.time
            self.air_start = data.time
            self.liftoffs += 1
        return contact

    def control(self, data):
        self.update_encoder(data)
        contact = self.update_state(data)
        # the GRF push only makes sense while the foot is really on the
        # ground; without this gate the Bezier force keeps whipping the leg
        # after liftoff and the impacts rectify it into backwards rotation
        if self.state == "STANCE" and contact:
            tau = self.stance(data, data.time - self.stance_start)
        else:
            tau = self.flight(data)
        engage = self._smooth01(data.time / SOFT_ENGAGE_TIME)
        return np.clip(engage * tau, -TORQUE_LIMIT, TORQUE_LIMIT)


# Link2 inertia without the counterweight (official URDF export values): the boom
# becomes leg-heavy and the robot sinks, which shows why the counterweight matters.
_LINK2_NO_CW = {
    "mass": 1.87654,
    "ipos": [-0.513966, 0.0016327, -0.00902703],
    "inertia": [0.0636508, 0.0630427, 0.00259304],
    "iquat": [0.500001, 0.500001, 0.499999, 0.499999],
}


def _remove_counterweight(model):
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link2")
    model.body_mass[b] = _LINK2_NO_CW["mass"]
    model.body_ipos[b] = _LINK2_NO_CW["ipos"]
    model.body_inertia[b] = _LINK2_NO_CW["inertia"]
    model.body_iquat[b] = _LINK2_NO_CW["iquat"]
    for name in ("cw_1", "cw_2", "cw_clamp"):
        g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        model.geom_rgba[g][3] = 0.0


def build_model_and_data(with_counterweight=True):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    if not with_counterweight:
        _remove_counterweight(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def main():
    args = [a for a in sys.argv[1:] if a != "--no-cw"]
    with_cw = "--no-cw" not in sys.argv
    default_name = "cad_hopping.mp4" if with_cw else "cad_hopping_nocw.mp4"
    out_path = Path(args[0]) if len(args) > 0 else ROOT / "results" / "renders" / default_name
    seconds = float(args[1]) if len(args) > 1 else 6.0

    model, data = build_model_and_data(with_counterweight=with_cw)
    hopper = Hopper(model)
    hopper.reset(data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [-0.15, 0.0, 0.20]
    cam.distance = 2.7
    cam.azimuth = 110
    cam.elevation = -28

    steps_per_frame = max(1, round((1.0 / FPS) / model.opt.timestep))
    frames = []

    for step in range(int(seconds / model.opt.timestep)):
        data.ctrl[:] = hopper.control(data)
        mujoco.mj_step(model, data)
        if step % steps_per_frame == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    renderer.close()
    out = write_video(frames, out_path, fps=FPS)
    cw = "with counterweight" if with_cw else "WITHOUT counterweight"
    print(f"Wrote {len(frames)} frames, {hopper.liftoffs} liftoffs ({cw}) -> {out}")


if __name__ == "__main__":
    main()
