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

TORQUE_LIMIT = 25.0
VELOCITY_FILTER_ALPHA = 0.15

MIN_STANCE_TIME = 0.14
MIN_FLIGHT_TIME = 0.12


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

def geom_position(model, data, geom_name_text):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name_text)
    return data.geom_xpos[geom_id].copy()

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
    kp_hip=45.0,
    kd_hip=4.5,
    kp_knee=38.0,
    kd_knee=4.0,
):
    hip = get_joint_qpos(model, data, "hip")
    knee = get_joint_qpos(model, data, "knee")

    hip_dof = model.jnt_dofadr[joint_id(model, "hip")]
    knee_dof = model.jnt_dofadr[joint_id(model, "knee")]

    hip_vel = data.qvel[hip_dof]
    knee_vel = data.qvel[knee_dof]

    tau_hip = kp_hip * (hip_ref - hip) + kd_hip * (hip_vel_ref - hip_vel)
    tau_knee = kp_knee * (knee_ref - knee) + kd_knee * (knee_vel_ref - knee_vel)

    return np.array([tau_hip, tau_knee])


def flight_control(model, data, foot_velocity_est):
    hip_ref = 0.45
    knee_ref = -0.80

    return joint_space_pd(
        model,
        data,
        hip_ref,
        knee_ref,
        kp_hip=8.0,
        kd_hip=1.0,
        kp_knee=6.0,
        kd_knee=1.0,
    )


def stance_control(model, data, stance_time):
    stance_duration = 0.35
    phase = np.clip(stance_time / stance_duration, 0.0, 1.0)

    compression_phase = 0.35

    hip_start = 0.45
    knee_start = -0.80

    hip_compressed = 0.35
    knee_compressed = -1.05

    hip_extended = 1.05
    knee_extended = -0.30

    if phase < compression_phase:
        local_phase = phase / compression_phase
        smooth_phase = 3.0 * local_phase**2 - 2.0 * local_phase**3

        hip_ref = hip_start + (hip_compressed - hip_start) * smooth_phase
        knee_ref = knee_start + (knee_compressed - knee_start) * smooth_phase
    else:
        local_phase = (phase - compression_phase) / (1.0 - compression_phase)
        smooth_phase = 3.0 * local_phase**2 - 2.0 * local_phase**3

        hip_ref = hip_compressed + (hip_extended - hip_compressed) * smooth_phase
        knee_ref = knee_compressed + (knee_extended - knee_compressed) * smooth_phase

    return joint_space_pd(
        model,
        data,
        hip_ref,
        knee_ref,
        kp_hip=50.0,
        kd_hip=5.0,
        kp_knee=45.0,
        kd_knee=4.5,
    )


def run_simulation():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    sim_time = 5.0

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

    rows = []

    while data.time < sim_time:
        foot_pos = site_position(model, data, "foot_site")
        hip_pos = body_position(model, data, "hip_body")

        foot_velocity_raw = (foot_pos - previous_foot_pos) / model.opt.timestep
        foot_velocity_est = (
            (1.0 - VELOCITY_FILTER_ALPHA) * foot_velocity_est
            + VELOCITY_FILTER_ALPHA * foot_velocity_raw
        )
        previous_foot_pos = foot_pos.copy()

        contact = foot_in_contact(model, data)
        time_in_state = data.time - state_start_time

        if state == FLIGHT and contact and time_in_state >= MIN_FLIGHT_TIME:
            state = STANCE
            state_start_time = data.time
            stance_start_time = data.time

        elif state == STANCE and (not contact) and time_in_state >= MIN_STANCE_TIME:
            state = FLIGHT
            state_start_time = data.time
            stance_start_time = None

        if state == FLIGHT:
            tau = flight_control(model, data, foot_velocity_est)
        else:
            stance_time = data.time - stance_start_time
            tau = stance_control(model, data, stance_time)

        tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

        data.ctrl[0] = tau[0]
        data.ctrl[1] = tau[1]

        hip_body_pos = body_position(model, data, "hip_body")
        hip_motor_pos = geom_position(model, data, "hip_motor_mass")

        rows.append({
            "time": data.time,
            "state": 1 if state == STANCE else 0,
            "gantry_pitch": get_joint_qpos(model, data, "gantry_pitch"),
            "gantry_pitch_vel": get_joint_qvel(model, data, "gantry_pitch"),
            "hip": get_joint_qpos(model, data, "hip"),
            "knee": get_joint_qpos(model, data, "knee"),
            "hip_body_world_z": hip_body_pos[2],
            "hip_motor_world_z": hip_motor_pos[2],
            "foot_world_x": foot_pos[0],
            "foot_world_y": foot_pos[1],
            "foot_world_z": foot_pos[2],
            "foot_rel_x": foot_pos[0] - hip_pos[0],
            "foot_rel_y": foot_pos[1] - hip_pos[1],
            "foot_rel_z": foot_pos[2] - hip_pos[2],
            "foot_vel_x_est": foot_velocity_est[0],
            "foot_vel_y_est": foot_velocity_est[1],
            "foot_vel_z_est": foot_velocity_est[2],
            "tau_hip": tau[0],
            "tau_knee": tau[1],
            "normal_force": normal_contact_force(model, data),
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
    gantry_pitch = np.array([row["gantry_pitch"] for row in rows])
    gantry_pitch_vel = np.array([row["gantry_pitch_vel"] for row in rows])
    hip = np.array([row["hip"] for row in rows])
    knee = np.array([row["knee"] for row in rows])
    hip_body_world_z = np.array([row["hip_body_world_z"] for row in rows])
    hip_motor_world_z = np.array([row["hip_motor_world_z"] for row in rows])
    foot_world_z = np.array([row["foot_world_z"] for row in rows])
    foot_rel_x = np.array([row["foot_rel_x"] for row in rows])
    foot_rel_y = np.array([row["foot_rel_y"] for row in rows])
    foot_rel_z = np.array([row["foot_rel_z"] for row in rows])
    foot_vel_z = np.array([row["foot_vel_z_est"] for row in rows])
    tau_hip = np.array([row["tau_hip"] for row in rows])
    tau_knee = np.array([row["tau_knee"] for row in rows])
    normal_force = np.array([row["normal_force"] for row in rows])

    plt.figure()
    plt.plot(time, gantry_pitch)
    plt.xlabel("Time [s]")
    plt.ylabel("Gantry pitch [rad]")
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "gantry_pitch.png", dpi=200)

    plt.figure()
    plt.plot(time, gantry_pitch_vel)
    plt.xlabel("Time [s]")
    plt.ylabel("Gantry pitch velocity [rad/s]")
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "gantry_pitch_velocity.png", dpi=200)

    plt.figure()
    plt.plot(time, hip, label="hip")
    plt.plot(time, knee, label="knee")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint position [rad]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "joint_positions.png", dpi=200)

    plt.figure()
    plt.plot(time, hip_body_world_z, label="hip body")
    plt.plot(time, hip_motor_world_z, label="hip motor mass")
    plt.xlabel("Time [s]")
    plt.ylabel("Hip / motor world height [m]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "hip_height.png", dpi=200)

    plt.figure()
    plt.plot(time, foot_rel_x, label="foot rel x")
    plt.plot(time, foot_rel_y, label="foot rel y")
    plt.plot(time, foot_rel_z, label="foot rel z")
    plt.xlabel("Time [s]")
    plt.ylabel("Foot position relative to hip [m]")
    plt.legend()
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "foot_relative_position.png", dpi=200)

    plt.figure()
    plt.plot(time, foot_vel_z)
    plt.xlabel("Time [s]")
    plt.ylabel("Estimated foot vertical velocity [m/s]")
    plt.grid(True)
    plt.savefig(PLOTS_DIR / "foot_vertical_velocity.png", dpi=200)

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