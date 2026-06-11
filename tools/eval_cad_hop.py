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
from cad_hop_controller import Hopper, TORQUE_LIMIT, build_model_and_data  # noqa: E402


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


def run(seconds=6.0, push_peak=None, stride=8):
    model, data = build_model_and_data()
    joint2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint2")
    initial_bias_joint2 = float(data.qfrc_bias[model.jnt_dofadr[joint2]])
    hopper = Hopper(model, push_peak=push_peak)
    hopper.reset(data)

    link3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link3")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot")
    lower_leg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "lower_leg_collision")
    body_meshes = mesh_points_for_body(model, "Link3", stride) + mesh_points_for_body(model, "Link4", stride)

    steps = int(seconds / model.opt.timestep)
    z_values = np.empty(steps)
    min_z = np.inf
    max_tau = np.zeros(2)
    finite = True
    foot_contact_steps = 0
    leg_contact_steps = 0
    both_contact_steps = 0

    for step in range(steps):
        tau = hopper.control(data)
        data.ctrl[:] = tau
        max_tau = np.maximum(max_tau, np.abs(tau))
        mujoco.mj_step(model, data)

        z_values[step] = data.xpos[link3, 2]
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

    real_air = [t for t in hopper.air_times if t > 0.10]
    if hopper.state == "FLIGHT" and hopper.air_start is not None:
        t = data.time - hopper.air_start
        if t > 0.10:
            real_air.append(t)

    return {
        "seconds": data.time,
        "bias_joint2": initial_bias_joint2,
        "link3_z_min": float(np.min(z_values[:step + 1])),
        "link3_z_max": float(np.max(z_values[:step + 1])),
        "link3_z_amp": float(np.max(z_values[:step + 1]) - np.min(z_values[:step + 1])),
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
    parser.add_argument("--stride", type=int, default=8)
    args = parser.parse_args()

    metrics = run(args.seconds, args.push, args.stride)
    print(f"seconds: {metrics['seconds']:.3f}")
    print(f"bias_joint2: {metrics['bias_joint2']:.4f} N*m")
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
