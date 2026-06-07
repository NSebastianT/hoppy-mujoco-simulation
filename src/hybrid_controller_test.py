import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"

FLIGHT = "FLIGHT"
STANCE = "STANCE"

L1 = 0.22
L2 = 0.22
DT = 0.001
TORQUE_LIMIT = 12.0


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
    data.qpos[qpos_id] = value


def set_joint_velocity(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qvel_id = model.jnt_dofadr[joint_id]
    data.qvel[qvel_id] = value


def get_joint_qpos(model, data, joint_name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
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


def foot_position(q):
    hip, knee = q

    x = -L1 * np.sin(hip) - L2 * np.sin(hip + knee)
    z = -L1 * np.cos(hip) - L2 * np.cos(hip + knee)

    return np.array([x, z])


def foot_jacobian(q):
    hip, knee = q

    dx_dhip = -L1 * np.cos(hip) - L2 * np.cos(hip + knee)
    dx_dknee = -L2 * np.cos(hip + knee)

    dz_dhip = L1 * np.sin(hip) + L2 * np.sin(hip + knee)
    dz_dknee = L2 * np.sin(hip + knee)

    return np.array([
        [dx_dhip, dx_dknee],
        [dz_dhip, dz_dknee],
    ])


def flight_control(q, qdot_est):
    p = foot_position(q)
    j = foot_jacobian(q)
    pdot = j @ qdot_est

    p_ref = np.array([0.00, -0.38])

    kp = np.diag([50.0, 80.0])
    kd = np.diag([2.0, 3.0])

    force = kp @ (p_ref - p) - kd @ pdot
    tau = j.T @ force

    return tau


def stance_control(q, stance_time):
    j = foot_jacobian(q)

    stance_duration = 0.15
    phase = np.clip(stance_time / stance_duration, 0.0, 1.0)

    vertical_force = 80.0 * (0.4 + 0.6 * np.sin(np.pi * phase))
    horizontal_force = 0.0

    desired_force = np.array([horizontal_force, vertical_force])
    tau = j.T @ desired_force

    return tau


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.50)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)

    set_joint_velocity(model, data, "gantry_pitch", 1.0)

    mujoco.mj_forward(model, data)

    q_prev = np.array([
        get_joint_qpos(model, data, "hip"),
        get_joint_qpos(model, data, "knee"),
    ])

    qdot_est = np.zeros(2)
    velocity_filter_alpha = 0.15

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

        while viewer.is_running() and time.time() - start_time < 15:
            q = np.array([
                get_joint_qpos(model, data, "hip"),
                get_joint_qpos(model, data, "knee"),
            ])

            qdot_raw = (q - q_prev) / DT
            qdot_est = (1.0 - velocity_filter_alpha) * qdot_est + velocity_filter_alpha * qdot_raw
            q_prev = q.copy()

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
                tau = flight_control(q, qdot_est)
            else:
                stance_time = data.time - stance_start_time
                tau = stance_control(q, stance_time)

            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            data.ctrl[0] = tau[0]
            data.ctrl[1] = tau[1]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.003)

    print("Hybrid controller test finished.")


if __name__ == "__main__":
    main()