"""Run the CAD hopper and write the rubric logs."""
from pathlib import Path
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from render_utils import select_gl_backend  # noqa: E402

select_gl_backend()

import mujoco  # noqa: E402
from cad_hop_controller import Hopper, TORQUE_LIMIT, build_model_and_data


ROOT = Path(__file__).resolve().parents[1]
LOG_PATH = ROOT / "results" / "logs" / "cad_states.csv"
PLOTS_DIR = ROOT / "results" / "plots"
SIM_TIME = 6.0


def joint_addresses(model):
    out = {}
    for name in ("joint1", "joint2", "joint3", "joint4"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        out[name] = (model.jnt_qposadr[jid], model.jnt_dofadr[jid])
    return out


def normal_contact_force(model, data, geom_names):
    total = 0.0
    force = np.zeros(6)
    floor = geom_names["floor"]
    contact_geoms = {geom_names["foot"], geom_names["lower_leg_collision"]}
    for i in range(data.ncon):
        pair = {data.contact[i].geom1, data.contact[i].geom2}
        if floor in pair and pair.intersection(contact_geoms):
            mujoco.mj_contactForce(model, data, i, force)
            total += abs(float(force[0]))
    return total


def foot_velocity(model, data, site_id, qvel=None):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return jacp @ (data.qvel if qvel is None else qvel)


def collect_row(model, data, hopper, joints, site_id, geom_names, tau):
    qvel_est = np.zeros(model.nv)
    qvel_est[joints["joint3"][1]] = hopper.v_est[0]
    qvel_est[joints["joint4"][1]] = hopper.v_est[1]
    foot = data.site_xpos[site_id].copy()
    foot_vel_true = foot_velocity(model, data, site_id)
    foot_vel_est = foot_velocity(model, data, site_id, qvel_est)

    row = {
        "t": data.time,
        "foot_x": foot[0],
        "foot_y": foot[1],
        "foot_z": foot[2],
        "foot_vx_true": foot_vel_true[0],
        "foot_vy_true": foot_vel_true[1],
        "foot_vz_true": foot_vel_true[2],
        "foot_vx_est": foot_vel_est[0],
        "foot_vy_est": foot_vel_est[1],
        "foot_vz_est": foot_vel_est[2],
        "contact_normal_force": normal_contact_force(model, data, geom_names),
        "hip_tau": tau[0],
        "knee_tau": tau[1],
        "hybrid_state": hopper.state,
        "hybrid_state_id": 1 if hopper.state == "STANCE" else 0,
    }

    for name in ("joint1", "joint2", "joint3", "joint4"):
        qadr, vadr = joints[name]
        row[f"{name}_pos"] = data.qpos[qadr]
        row[f"{name}_vel_true"] = data.qvel[vadr]

    row["joint1_vel_est"] = np.nan
    row["joint2_vel_est"] = np.nan
    row["joint3_vel_est"] = hopper.v_est[0]
    row["joint4_vel_est"] = hopper.v_est[1]
    return row


def write_csv(rows):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_lines(t, series, title, ylabel, path):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    for label, values in series:
        ax.plot(t, values, label=label, linewidth=1.2)
    ax.set_title(title)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def make_plots(rows):
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    t = np.array([r["t"] for r in rows])

    plot_lines(
        t,
        [(name, np.array([r[f"{name}_pos"] for r in rows])) for name in ("joint1", "joint2", "joint3", "joint4")],
        "Posiciones articulares CAD",
        "q [rad]",
        PLOTS_DIR / "cad_joint_positions.png",
    )
    plot_lines(
        t,
        [(axis, np.array([r[f"foot_{axis}"] for r in rows])) for axis in ("x", "y", "z")],
        "Posicion cartesiana del pie",
        "pos [m]",
        PLOTS_DIR / "cad_foot_position.png",
    )
    plot_lines(
        t,
        [
            ("joint3 true", np.array([r["joint3_vel_true"] for r in rows])),
            ("joint3 encoder", np.array([r["joint3_vel_est"] for r in rows])),
            ("joint4 true", np.array([r["joint4_vel_true"] for r in rows])),
            ("joint4 encoder", np.array([r["joint4_vel_est"] for r in rows])),
        ],
        "Velocidades articulares",
        "dq [rad/s]",
        PLOTS_DIR / "cad_joint_velocities.png",
    )
    plot_lines(
        t,
        [
            ("vx true", np.array([r["foot_vx_true"] for r in rows])),
            ("vx encoder", np.array([r["foot_vx_est"] for r in rows])),
            ("vy true", np.array([r["foot_vy_true"] for r in rows])),
            ("vy encoder", np.array([r["foot_vy_est"] for r in rows])),
            ("vz true", np.array([r["foot_vz_true"] for r in rows])),
            ("vz encoder", np.array([r["foot_vz_est"] for r in rows])),
        ],
        "Velocidad cartesiana del pie",
        "vel [m/s]",
        PLOTS_DIR / "cad_foot_velocity.png",
    )
    plot_lines(
        t,
        [("normal", np.array([r["contact_normal_force"] for r in rows]))],
        "Fuerza normal de contacto",
        "F [N]",
        PLOTS_DIR / "cad_contact_force.png",
    )
    plot_lines(
        t,
        [
            ("hip", np.array([r["hip_tau"] for r in rows])),
            ("knee", np.array([r["knee_tau"] for r in rows])),
            ("hip limit", np.full_like(t, TORQUE_LIMIT[0])),
            ("knee limit", np.full_like(t, TORQUE_LIMIT[1])),
            ("hip -limit", np.full_like(t, -TORQUE_LIMIT[0])),
            ("knee -limit", np.full_like(t, -TORQUE_LIMIT[1])),
        ],
        "Torques de control",
        "tau [N m]",
        PLOTS_DIR / "cad_torques.png",
    )
    plot_lines(
        t,
        [("STANCE", np.array([r["hybrid_state_id"] for r in rows]))],
        "Estado hibrido",
        "0=FLIGHT, 1=STANCE",
        PLOTS_DIR / "cad_hybrid_state.png",
    )


def run(seconds=SIM_TIME):
    model, data = build_model_and_data()
    hopper = Hopper(model)
    hopper.reset(data)

    joints = joint_addresses(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    geom_names = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ("floor", "foot", "lower_leg_collision")
    }

    rows = []
    steps = int(seconds / model.opt.timestep)
    for _ in range(steps):
        tau = hopper.control(data)
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        rows.append(collect_row(model, data, hopper, joints, site_id, geom_names, tau.copy()))

    write_csv(rows)
    make_plots(rows)
    return rows


def main():
    rows = run()
    print(f"CSV: {LOG_PATH} ({len(rows)} rows)")
    print(f"Plots: {PLOTS_DIR}")


if __name__ == "__main__":
    main()
