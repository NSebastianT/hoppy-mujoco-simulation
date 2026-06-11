"""Evaluate the CAD hopper without rendering."""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from render_utils import select_gl_backend  # noqa: E402

select_gl_backend()

import mujoco  # noqa: E402
from cad_hop_controller import HORIZONTAL_FORCE, Hopper, TORQUE_LIMIT, build_model_and_data  # noqa: E402


def mesh_points_for_body(model, body_name, stride=8):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    points = []
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] != body_id:
            continue
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = model.geom_dataid[geom_id]
        start = model.mesh_vertadr[mesh_id]
        count = model.mesh_vertnum[mesh_id]
        verts = model.mesh_vert[start:start + count:stride].copy()
        mesh_mat = np.zeros(9)
        mujoco.mju_quat2Mat(mesh_mat, model.mesh_quat[mesh_id])
        mesh_mat = mesh_mat.reshape(3, 3)
        verts = model.mesh_pos[mesh_id] + verts @ mesh_mat.T
        points.append((geom_id, verts))
    return points


def min_mesh_z(model, data, body_meshes):
    zmin = np.inf
    for geom_id, verts in body_meshes:
        mat = data.geom_xmat[geom_id].reshape(3, 3)
        pos = data.geom_xpos[geom_id]
        world = pos + verts @ mat.T
        zmin = min(zmin, float(np.min(world[:, 2])))
    return zmin


def run(seconds=6.0, push_peak=None, horizontal_force=HORIZONTAL_FORCE, stride=8):
    model, data = build_model_and_data()
    joint1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
    joint2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint2")
    joint1_q = model.jnt_qposadr[joint1]
    initial_bias_joint2 = float(data.qfrc_bias[model.jnt_dofadr[joint2]])
    initial_yaw = float(data.qpos[joint1_q])
    hopper = Hopper(model, push_peak=push_peak, horizontal_force=horizontal_force)
    hopper.reset(data)

    link3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link3")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    lower_leg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "lower_leg_collision")
    body_meshes = mesh_points_for_body(model, "Link3", stride) + mesh_points_for_body(model, "Link4", stride)

    steps = int(seconds / model.opt.timestep)
    z_values = np.empty(steps)
    yaw_values = np.empty(steps)
    min_z = np.inf
    max_tau = np.zeros(2)
    finite = True
    foot_contact_steps = 0
    leg_contact_steps = 0
    both_contact_steps = 0
    startup_qvel_max = 0.0
    flight_peaks = []
    current_flight_peak = None

    for step in range(steps):
        tau = hopper.control(data)
        data.ctrl[:] = tau
        max_tau = np.maximum(max_tau, np.abs(tau))
        mujoco.mj_step(model, data)
        if data.time <= 0.2:
            startup_qvel_max = max(startup_qvel_max, float(np.max(np.abs(data.qvel))))

        z_values[step] = data.xpos[link3, 2]
        yaw_values[step] = data.qpos[joint1_q]
        if hopper.state == "FLIGHT" and hopper.liftoffs > 0:
            if current_flight_peak is None:
                current_flight_peak = z_values[step]
            else:
                current_flight_peak = max(current_flight_peak, z_values[step])
        elif current_flight_peak is not None:
            flight_peaks.append(float(current_flight_peak))
            current_flight_peak = None
        foot_contact = False
        leg_contact = False
        for i in range(data.ncon):
            pair = {data.contact[i].geom1, data.contact[i].geom2}
            foot_contact = foot_contact or pair == {foot, floor}
            leg_contact = leg_contact or pair == {lower_leg, floor}
        foot_contact_steps += int(foot_contact)
        leg_contact_steps += int(leg_contact)
        both_contact_steps += int(foot_contact and leg_contact)

        if step % stride == 0:
            min_z = min(min_z, min_mesh_z(model, data, body_meshes))
        if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
            finite = False
            break

    if current_flight_peak is not None:
        flight_peaks.append(float(current_flight_peak))

    z_values = z_values[:step + 1]
    yaw_values = yaw_values[:step + 1]
    yaw_delta = float(yaw_values[-1] - initial_yaw)
    yaw_steps = np.diff(yaw_values)
    yaw_progress = float(np.sum(np.abs(yaw_steps)))
    yaw_monotonic_fraction = 1.0
    if yaw_steps.size and abs(yaw_delta) > 1e-9:
        yaw_monotonic_fraction = float(np.mean(np.sign(yaw_delta) * yaw_steps >= -1e-4))

    first_hop_peak = flight_peaks[0] if flight_peaks else float("nan")
    steady_peak = float(np.mean(flight_peaks[1:])) if len(flight_peaks) > 1 else float("nan")
    peak_ratio = first_hop_peak / steady_peak if steady_peak and np.isfinite(steady_peak) else float("nan")

    real_air = [t for t in hopper.air_times if t > 0.10]
    if hopper.state == "FLIGHT" and hopper.air_start is not None:
        t = data.time - hopper.air_start
        if t > 0.10:
            real_air.append(t)

    return {
        "seconds": data.time,
        "bias_joint2": initial_bias_joint2,
        "yaw_delta": yaw_delta,
        "yaw_progress": yaw_progress,
        "yaw_min": float(np.min(yaw_values)),
        "yaw_max": float(np.max(yaw_values)),
        "yaw_monotonic_fraction": yaw_monotonic_fraction,
        "startup_qvel_max": startup_qvel_max,
        "first_hop_peak": float(first_hop_peak),
        "steady_peak": float(steady_peak),
        "first_to_steady_peak_ratio": float(peak_ratio),
        "flight_peaks": flight_peaks,
        "link3_z_min": float(np.min(z_values)),
        "link3_z_max": float(np.max(z_values)),
        "link3_z_amp": float(np.max(z_values) - np.min(z_values)),
        "mesh_min_z": float(min_z),
        "real_hops": len(real_air),
        "air_times": real_air,
        "finite": finite,
        "max_tau": max_tau.tolist(),
        "within_torque": bool(np.all(max_tau <= TORQUE_LIMIT + 1e-9)),
        "liftoffs": hopper.liftoffs,
        "foot_contact_steps": foot_contact_steps,
        "leg_contact_steps": leg_contact_steps,
        "both_contact_steps": both_contact_steps,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--push", type=float, default=None)
    parser.add_argument("--horizontal-force", type=float, default=HORIZONTAL_FORCE)
    parser.add_argument("--stride", type=int, default=8)
    args = parser.parse_args()

    metrics = run(args.seconds, args.push, args.horizontal_force, args.stride)
    print(f"seconds: {metrics['seconds']:.3f}")
    print(f"bias_joint2: {metrics['bias_joint2']:.4f} N*m")
    print(f"yaw_delta: {metrics['yaw_delta']:.4f} rad")
    print(f"yaw_progress: {metrics['yaw_progress']:.4f} rad")
    print(f"yaw_range: {metrics['yaw_min']:.4f} .. {metrics['yaw_max']:.4f} rad")
    print(f"yaw_monotonic_fraction: {metrics['yaw_monotonic_fraction']:.3f}")
    print(f"startup_qvel_max_0p2s: {metrics['startup_qvel_max']:.4f} rad/s")
    print(f"first_hop_peak: {metrics['first_hop_peak']:.4f} m")
    print(f"steady_peak: {metrics['steady_peak']:.4f} m")
    print(f"first_to_steady_peak_ratio: {metrics['first_to_steady_peak_ratio']:.3f}")
    print("flight_peaks:", ", ".join(f"{z:.3f}" for z in metrics["flight_peaks"]) or "none")
    print(f"link3_z_amp: {metrics['link3_z_amp']:.4f} m")
    print(f"link3_z_range: {metrics['link3_z_min']:.4f} .. {metrics['link3_z_max']:.4f} m")
    print(f"mesh_min_z: {metrics['mesh_min_z']:.4f} m")
    print(f"real_hops: {metrics['real_hops']} (air_times > 0.10 s)")
    print("air_times:", ", ".join(f"{t:.3f}" for t in metrics["air_times"]) or "none")
    print(f"finite: {metrics['finite']}")
    print(f"max_tau: hip={metrics['max_tau'][0]:.3f}, knee={metrics['max_tau'][1]:.3f} N*m")
    print(f"within_torque: {metrics['within_torque']}")
    print(f"liftoffs: {metrics['liftoffs']}")
    print(f"foot_contact_steps: {metrics['foot_contact_steps']}")
    print(f"leg_contact_steps: {metrics['leg_contact_steps']}")
    print(f"both_contact_steps: {metrics['both_contact_steps']}")


if __name__ == "__main__":
    main()
