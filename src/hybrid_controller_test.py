import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"

FLIGHT = "FLIGHT"
STANCE = "STANCE"

TORQUE_LIMIT = 12.0
VELOCITY_FILTER_ALPHA = 0.15


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
    tau = jacobian.T @ desired_force

    return tau


def stance_control(model, data, stance_time):
    stance_duration = 0.15
    phase = np.clip(stance_time / stance_duration, 0.0, 1.0)

    vertical_force = 80.0 * (0.4 + 0.6 * np.sin(np.pi * phase))
    desired_force = np.array([0.0, 0.0, vertical_force])

    jacobian = foot_jacobian_for_actuated_joints(model, data)
    tau = jacobian.T @ desired_force

    return tau


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.50)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)

    set_joint_velocity(model, data, "gantry_pitch", 1.0)

    mujoco.mj_forward(model, data)

    previous_foot_pos = site_position(model, data, "foot_site")
    foot_velocity_est = np.zeros(3)

    state = STANCE if foot_in_contact(model, data) else FLIGHT
    stance_start_time = data.time if state == STANCE else None

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
            new_state = STANCE if contact else FLIGHT

            if new_state != state:
                print(f"{state} -> {new_state} at t = {data.time:.3f} s")
                state = new_state

                if state == STANCE:
                    stance_start_time = data.time
                else:
                    stance_start_time = None

            if state == FLIGHT:
                tau = flight_control(model, data, foot_velocity_est)
            else:
                stance_time = data.time - stance_start_time
                tau = stance_control(model, data, stance_time)

            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            data.ctrl[0] = tau[0]
            data.ctrl[1] = tau[1]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.003)

    print("Hybrid controller test finished.")


if __name__ == "__main__":
    main()