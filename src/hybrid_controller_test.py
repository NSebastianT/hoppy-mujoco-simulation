import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"

FLIGHT = "FLIGHT"
STANCE = "STANCE"

TORQUE_LIMIT = 35.0
VELOCITY_FILTER_ALPHA = 0.15

MIN_STANCE_TIME = 0.10
MAX_STANCE_TIME = 0.28
MIN_FLIGHT_TIME = 0.16

TARGET_HIP_APEX_Z = 0.48
PUSH_FORCE_MIN = 180.0
PUSH_FORCE_MAX = 420.0
PUSH_FORCE_GAIN = 650.0

CONTROL_MEMORY = {
    "push_force": 220.0,
    "flight_apex_z": 0.0,
    "hop_count": 0,
}


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


def get_joint_qvel(model, data, joint_name):
    jid = joint_id(model, joint_name)
    qvel_id = model.jnt_dofadr[jid]
    return data.qvel[qvel_id]


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


def joint_space_pd(
    model,
    data,
    hip_ref,
    knee_ref,
    hip_vel_ref=0.0,
    knee_vel_ref=0.0,
    kp_hip=8.0,
    kd_hip=1.0,
    kp_knee=6.0,
    kd_knee=1.0,
):
    hip = get_joint_qpos(model, data, "hip")
    knee = get_joint_qpos(model, data, "knee")

    hip_vel = get_joint_qvel(model, data, "hip")
    knee_vel = get_joint_qvel(model, data, "knee")

    tau_hip = kp_hip * (hip_ref - hip) + kd_hip * (hip_vel_ref - hip_vel)
    tau_knee = kp_knee * (knee_ref - knee) + kd_knee * (knee_vel_ref - knee_vel)

    return np.array([tau_hip, tau_knee])


def flight_control(model, data):
    hip_ref = 0.45
    knee_ref = -0.80

    return joint_space_pd(
        model,
        data,
        hip_ref,
        knee_ref,
        kp_hip=6.0,
        kd_hip=1.0,
        kp_knee=4.5,
        kd_knee=1.0,
    )


def stance_control(model, data, stance_time):
    J = foot_jacobian_for_actuated_joints(model, data)

    ramp_time = 0.12
    ramp = np.clip(stance_time / ramp_time, 0.0, 1.0)
    ramp = 3.0 * ramp**2 - 2.0 * ramp**3

    vertical_push_force = CONTROL_MEMORY["push_force"] * ramp

    desired_foot_force = np.array([0.0, 0.0, -vertical_push_force])
    tau_push = J.T @ desired_foot_force

    hip_ref = 0.45
    knee_ref = -0.90

    tau_posture = joint_space_pd(
        model,
        data,
        hip_ref,
        knee_ref,
        kp_hip=10.0,
        kd_hip=1.5,
        kp_knee=12.0,
        kd_knee=1.8,
    )

    return tau_push + tau_posture


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.00)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)

    set_joint_velocity(model, data, "gantry_pitch", 0.50)

    mujoco.mj_forward(model, data)

    previous_foot_pos = site_position(model, data, "foot_site")
    foot_velocity_est = np.zeros(3)

    state = STANCE if foot_in_contact(model, data) else FLIGHT
    state_start_time = data.time
    stance_start_time = data.time if state == STANCE else None

    max_abs_tau_hip = 0.0
    max_abs_tau_knee = 0.0

    hip_body_pos = body_position(model, data, "hip_body")
    CONTROL_MEMORY["flight_apex_z"] = hip_body_pos[2]

    print(f"Initial state: {state}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.25, 0.0, 0.35]
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.sync()

        input("Viewer ready. Press Enter to start simulation...")

        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < 60:
            foot_pos = site_position(model, data, "foot_site")

            foot_velocity_raw = (foot_pos - previous_foot_pos) / model.opt.timestep
            foot_velocity_est = (
                (1.0 - VELOCITY_FILTER_ALPHA) * foot_velocity_est
                + VELOCITY_FILTER_ALPHA * foot_velocity_raw
            )
            previous_foot_pos = foot_pos.copy()

            contact = foot_in_contact(model, data)
            time_in_state = data.time - state_start_time

            hip_body_z = body_position(model, data, "hip_body")[2]

            if state == FLIGHT:
                CONTROL_MEMORY["flight_apex_z"] = max(
                    CONTROL_MEMORY["flight_apex_z"],
                    hip_body_z,
                )

            if state == FLIGHT and contact and time_in_state >= MIN_FLIGHT_TIME:
                if CONTROL_MEMORY["hop_count"] > 0:
                    height_error = TARGET_HIP_APEX_Z - CONTROL_MEMORY["flight_apex_z"]

                    new_push_force = (
                        CONTROL_MEMORY["push_force"]
                        + PUSH_FORCE_GAIN * height_error
                    )

                    CONTROL_MEMORY["push_force"] = float(
                        np.clip(new_push_force, PUSH_FORCE_MIN, PUSH_FORCE_MAX)
                    )

                    print(
                        f"apex_z = {CONTROL_MEMORY['flight_apex_z']:.3f} m, "
                        f"push_force = {CONTROL_MEMORY['push_force']:.1f} N"
                    )

                print(f"FLIGHT -> STANCE at t = {data.time:.3f} s")
                state = STANCE
                state_start_time = data.time
                stance_start_time = data.time

            elif state == STANCE and (
                ((not contact) and time_in_state >= MIN_STANCE_TIME)
                or time_in_state >= MAX_STANCE_TIME
            ):
                print(f"STANCE -> FLIGHT at t = {data.time:.3f} s")

                CONTROL_MEMORY["hop_count"] += 1
                CONTROL_MEMORY["flight_apex_z"] = hip_body_z

                state = FLIGHT
                state_start_time = data.time
                stance_start_time = None

            if state == FLIGHT:
                tau = flight_control(model, data)
            else:
                stance_time = data.time - stance_start_time
                tau = stance_control(model, data, stance_time)

            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            max_abs_tau_hip = max(max_abs_tau_hip, abs(tau[0]))
            max_abs_tau_knee = max(max_abs_tau_knee, abs(tau[1]))

            data.ctrl[0] = tau[0]
            data.ctrl[1] = tau[1]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.003)

    print(f"Max |tau_hip| commanded: {max_abs_tau_hip:.3f} Nm")
    print(f"Max |tau_knee| commanded: {max_abs_tau_knee:.3f} Nm")
    print("Hybrid controller test finished.")


if __name__ == "__main__":
    main()