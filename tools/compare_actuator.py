"""Fase 2: simulaciones comparativas del modelo de actuador.

Corre el mismo controlador de salto sobre el modelo CAD toggleando cada efecto
(armature de inercia reflejada, damping back-EMF, resorte paralelo de rodilla y
saturacion de torque) y grafica la altura del cuerpo, con una tabla resumen.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

import cad_hop_controller as C

ROOT = Path(__file__).resolve().parents[1]
PLOTS = ROOT / "results" / "plots"
SECONDS = 5.0


def run(override, torque_limit=None):
    model, data = C.build_model_and_data()
    j3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint3")
    j4 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint4")
    ids = {
        "hip_dof": model.jnt_dofadr[j3], "knee_dof": model.jnt_dofadr[j4], "knee_jnt": j4,
        "hip_act": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip_motor"),
        "knee_act": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee_motor"),
    }
    override(model, ids)
    link3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link3")

    saved_limit = C.TORQUE_LIMIT
    if torque_limit is not None:
        C.TORQUE_LIMIT = np.array(torque_limit)
    try:
        hopper = C.Hopper(model)
        hopper.reset(data)
        t, z, max_tau, unstable = [], [], 0.0, False
        for _ in range(int(SECONDS / model.opt.timestep)):
            data.ctrl[:] = hopper.control(data)
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all():
                unstable = True
                break
            t.append(data.time)
            z.append(data.xpos[link3][2])
            max_tau = max(max_tau, float(np.abs(data.ctrl).max()))
    finally:
        C.TORQUE_LIMIT = saved_limit

    z = np.array(z)
    return {
        "t": np.array(t), "z": z, "stable": not unstable, "hops": hopper.liftoffs,
        "peak": float(z.max()) if z.size else float("nan"), "max_tau": max_tau,
    }


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)

    def baseline(m, ids):
        pass

    def no_armature(m, ids):
        m.dof_armature[ids["hip_dof"]] = 0.0
        m.dof_armature[ids["knee_dof"]] = 0.0

    def no_damping(m, ids):
        m.dof_damping[ids["hip_dof"]] = 0.0
        m.dof_damping[ids["knee_dof"]] = 0.0

    def no_spring(m, ids):
        m.jnt_stiffness[ids["knee_jnt"]] = 0.0

    def no_saturation(m, ids):
        m.actuator_ctrlrange[ids["hip_act"]] = [-35.0, 35.0]
        m.actuator_ctrlrange[ids["knee_act"]] = [-35.0, 35.0]

    cases = [
        ("baseline", baseline, None),
        ("sin armature", no_armature, None),
        ("sin damping", no_damping, None),
        ("sin resorte rodilla", no_spring, None),
        ("sin saturacion (+-35 Nm)", no_saturation, [35.0, 35.0]),
    ]

    results = {name: run(ov, tl) for name, ov, tl in cases}

    plt.figure(figsize=(9, 5))
    for name, _, _ in cases:
        r = results[name]
        plt.plot(r["t"], r["z"], label=name)
    plt.xlabel("t [s]")
    plt.ylabel("altura del cuerpo (Link3 z) [m]")
    plt.title("Fase 2: influencia de armature, damping, resorte y saturacion")
    plt.legend()
    plt.grid(True)
    out = PLOTS / "cad_comparison_height.png"
    plt.savefig(out, dpi=110, bbox_inches="tight")

    print(f"{'caso':26} {'estable':8} {'saltos':7} {'pico z [m]':11} {'max|tau| [Nm]':12}")
    for name, _, _ in cases:
        r = results[name]
        print(f"{name:26} {str(r['stable']):8} {r['hops']:<7} {r['peak']:<11.3f} {r['max_tau']:<12.2f}")
    print(f"\nplot -> {out}")


if __name__ == "__main__":
    main()
