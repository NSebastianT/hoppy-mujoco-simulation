from pathlib import Path

import csv
import mujoco
import numpy as np
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"
LOG_PATH = ROOT / "results" / "logs" / "hybrid_log.csv"
PLOTS_DIR = ROOT / "results" / "plots"

FLIGHT = "FLIGHT"
STANCE = "STANCE"

L1 = 0.22
L2 = 0.22
TORQUE_LIMIT = 12.0


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
    data.qpos[qpos_id] = value


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


def normal_contact_force(model, data):
    total_force = 0.0
    force_buffer = np.zeros(6)

    for i in range(data.ncon):
        contact = data.contact[i]
        name1 = geom_name(model, contact.geom1)
        name2 = geom_name(model, contact.geom2)

        if {"foot", "floor"} == {name1, name2}:
            mujoco.mj_contactForce(model, data, i, force_buffer)
            total_force += abs(force_buffer[0])

    return total_force


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


def run_simulation():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    sim_time = 5.0

    set_joint(model, data, "gantry_pitch", 0.55)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    mujoco.mj_forward(model, data)

    q_prev = np.array([
        get_joint_qpos(model, data, "hip"),
        get_joint_qpos(model, data, "knee"),
    ])

    qdot_est = np.zeros(2)
    velocity_filter_alpha = 0.15

    state = STANCE if foot_in_contact(model, data) else FLIGHT
    stance_start_time = data.time if state == STANCE else None

    rows = []

    while data.time < sim_time:
        q = np.array([
            get_joint_qpos(model, data, "hip"),
            get_joint_qpos(model, data, "knee"),
        ])

        qdot_raw = (q - q_prev) / dt
        qdot_est = (1.0 - velocity_filter_alpha) * qdot_est + velocity_filter_alpha * qdot_raw
        q_prev = q.copy()

        contact = foot_in_contact(model, data)
        new_state = STANCE if contact else FLIGHT

        if new_state != state:
            state = new_state
            stance_start_time = data.time if state == STANCE else None

        if state == FLIGHT:
            tau = flight_control(q, qdot_est)
        else:
            stance_time = data.time - stance_start_time
            tau = stance_control(q, stance_time)

        tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

        data.ctrl[0] = tau[0]
        data.ctrl[1] = tau[1]

        foot_pos = foot_position(q)
        contact_force = normal_contact_force(model, data)

        rows.append({
            "time": data.time,
            "state": 1 if state == STANCE else 0,
            "hip": q[0],
            "knee": q[1],
            "hip_vel_est": qdot_est[0],
            "knee_vel_est": qdot_est[1],
            "foot_x": foot_pos[0],
            "foot_z": foot_pos[1],
            "tau_hip": tau[0],
            "tau_knee": tau[1],
            "normal_force": contact_force,
        })

        mujoco.mj_step(model, data)

    return rows


def save_log(rows):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)

    with open(LOG_PATH, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved log: {LOG_PATH}")


def plot_results(rows):
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    time = np.array([row["time"] for row in rows])
    state = np.array([row["state"] for row in rows])
    hip = np.array([row["hip"] for row in rows])
    knee = np.array([row["knee"] for row in rows])
    hip_vel = np.array([row["hip_vel_est"] for row in rows])
    knee_vel = np.array([row["knee_vel_est"] for row in rows])
    foot_x = np.array([row["foot_x"] for row in rows])
    foot_z = np.array([row["foot_z"] for row in rows])
    tau_hip = np.array([row["tau_hip"] for row in rows])
    tau_knee = np.array([row["tau_knee"] for row in rows])
    normal_force = np.array([row["normal_force"] for row in rows])

    plt.figure()
    plt.plot(time, hip, label="hip")
    plt.plot(time, knee, label="knee")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint position [rad]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "joint_positions.png", dpi=200)

    plt.figure()
    plt.plot(time, hip_vel, label="hip estimated velocity")
    plt.plot(time, knee_vel, label="knee estimated velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint velocity [rad/s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "estimated_velocities.png", dpi=200)

    plt.figure()
    plt.plot(time, foot_x, label="foot x")
    plt.plot(time, foot_z, label="foot z")
    plt.xlabel("Time [s]")
    plt.ylabel("Foot position [m]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "foot_position.png", dpi=200)

    plt.figure()
    plt.plot(time, tau_hip, label="hip torque")
    plt.plot(time, tau_knee, label="knee torque")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque command [Nm]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "torques.png", dpi=200)

    plt.figure()
    plt.plot(time, normal_force)
    plt.xlabel("Time [s]")
    plt.ylabel("Normal contact force [N]")
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "normal_force.png", dpi=200)

    plt.figure()
    plt.plot(time, state)
    plt.xlabel("Time [s]")
    plt.ylabel("State: 0=FLIGHT, 1=STANCE")
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "hybrid_state.png", dpi=200)

    print(f"Saved plots in: {PLOTS_DIR}")


def main():
    rows = run_simulation()
    save_log(rows)
    plot_results(rows)
    print("Logged simulation finished.")


if __name__ == "__main__":
    main()