from pathlib import Path
import re

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"

FLIGHT = "FLIGHT"
STANCE = "STANCE"

TORQUE_LIMIT = 12.0
VELOCITY_FILTER_ALPHA = 0.15


def make_model_xml(counterweight_mass):
    xml = MODEL_PATH.read_text(encoding="utf-8")

    pattern = r'(<geom name="counterweight"[^>]*mass=")[^"]+(")'
    replacement = rf'\g<1>{counterweight_mass}\2'

    xml_new, count = re.subn(pattern, replacement, xml, flags=re.DOTALL)

    if count != 1:
        raise RuntimeError("Could not replace counterweight mass. Check models/hoppy.xml.")

    return xml_new


def joint_id(model, joint_name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)


def set_joint(model, data, joint_name, value):
    jid = joint_id(model, joint_name)
    qpos_id = model.jnt_qposadr[jid]
    data.qpos[qpos_id] = value


def set_joint_velocity(model, data, joint_name, value):
    jid = joint_id(model, joint_name)
    qvel_id = model.jnt_dofadr[jid]
    data.qvel[qvel_id] = value


def get_joint_qpos(model, data, joint_name):
    jid = joint_id(model, joint_name)
    qpos_id = model.jnt_qposadr[jid]
    return data.qpos[qpos_id]


def geom_name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def foot_in_contact(model, data):
    for i in range(data.ncon):
        contact = data.contact[i]
        name1 = geom_name(model, contact.geom1)
        name2 = geom_name(model, contact.geom2)

        if {"foot", "floor"} == {name1, name2}:
            return True

    return False


def site_position(model, data, site_name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    return data.site_xpos[site_id].copy()


def body_position(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[body_id].copy()


def foot_jacobian_for_actuated_joints(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    hip_dof = model.jnt_dofadr[joint_id(model, "hip")]
    knee_dof = model.jnt_dofadr[joint_id(model, "knee")]

    return jacp[:, [hip_dof, knee_dof]]


def flight_control(model, data, foot_velocity_est):
    foot_pos = site_position(model, data, "foot_site")
    hip_pos = body_position(model, data, "hip_body")

    foot_ref = hip_pos + np.array([0.0, 0.0, -0.38])

    kp = np.diag([20.0, 20.0, 80.0])
    kd = np.diag([1.0, 1.0, 3.0])

    desired_force = kp @ (foot_ref - foot_pos) - kd @ foot_velocity_est

    jacobian = foot_jacobian_for_actuated_joints(model, data)
    return jacobian.T @ desired_force


def stance_control(model, data, stance_time, force_scale):
    stance_duration = 0.20
    phase = np.clip(stance_time / stance_duration, 0.0, 1.0)

    vertical_force = force_scale * (0.4 + 0.6 * np.sin(np.pi * phase))
    desired_force = np.array([0.0, 0.0, vertical_force])

    jacobian = foot_jacobian_for_actuated_joints(model, data)
    return jacobian.T @ desired_force


def simulate(counterweight_mass, force_scale):
    xml = make_model_xml(counterweight_mass)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.00)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    set_joint_velocity(model, data, "gantry_pitch", 0.50)

    mujoco.mj_forward(model, data)

    previous_foot_pos = site_position(model, data, "foot_site")
    foot_velocity_est = np.zeros(3)

    state = STANCE if foot_in_contact(model, data) else FLIGHT
    stance_start_time = data.time if state == STANCE else None

    touchdown_count = 0
    liftoff_count = 0
    contact_steps = 0
    total_steps = 0

    min_z = site_position(model, data, "foot_site")[2]
    max_z = min_z

    min_pitch = get_joint_qpos(model, data, "gantry_pitch")
    max_pitch = min_pitch

    previous_contact = foot_in_contact(model, data)

    while data.time < 2.0:
        foot_pos = site_position(model, data, "foot_site")

        foot_velocity_raw = (foot_pos - previous_foot_pos) / model.opt.timestep
        foot_velocity_est = (
            (1.0 - VELOCITY_FILTER_ALPHA) * foot_velocity_est
            + VELOCITY_FILTER_ALPHA * foot_velocity_raw
        )
        previous_foot_pos = foot_pos.copy()

        contact = foot_in_contact(model, data)
        new_state = STANCE if contact else FLIGHT

        if contact and not previous_contact:
            touchdown_count += 1

        if previous_contact and not contact:
            liftoff_count += 1

        previous_contact = contact

        if contact:
            contact_steps += 1

        if new_state != state:
            state = new_state
            stance_start_time = data.time if state == STANCE else None

        if state == FLIGHT:
            tau = flight_control(model, data, foot_velocity_est)
        else:
            stance_time = data.time - stance_start_time
            tau = stance_control(model, data, stance_time, force_scale)

        tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

        data.ctrl[0] = tau[0]
        data.ctrl[1] = tau[1]

        mujoco.mj_step(model, data)

        z = site_position(model, data, "foot_site")[2]
        pitch = get_joint_qpos(model, data, "gantry_pitch")

        min_z = min(min_z, z)
        max_z = max(max_z, z)

        min_pitch = min(min_pitch, pitch)
        max_pitch = max(max_pitch, pitch)

        total_steps += 1

    contact_fraction = contact_steps / total_steps

    return {
        "cw": counterweight_mass,
        "force": force_scale,
        "td": touchdown_count,
        "lo": liftoff_count,
        "contact_percent": 100.0 * contact_fraction,
        "min_z": min_z,
        "max_z": max_z,
        "min_pitch": min_pitch,
        "max_pitch": max_pitch,
    }


def main():
    masses = [0.60, 0.80, 1.00, 1.20, 1.40]
    forces = [25.0, 40.0, 60.0, 80.0]

    print("cw_kg | Fz_scale | TD | LO | contact_% | min_z | max_z | pitch_min | pitch_max")
    print("-------------------------------------------------------------------------------")

    for mass in masses:
        for force in forces:
            r = simulate(mass, force)

            print(
                f"{r['cw']:>5.2f} | "
                f"{r['force']:>8.1f} | "
                f"{r['td']:>2d} | "
                f"{r['lo']:>2d} | "
                f"{r['contact_percent']:>9.2f} | "
                f"{r['min_z']:>5.3f} | "
                f"{r['max_z']:>5.3f} | "
                f"{r['min_pitch']:>9.3f} | "
                f"{r['max_pitch']:>9.3f}"
            )

        print()


if __name__ == "__main__":
    main()