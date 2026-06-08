from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


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


def foot_world_z(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    return data.site_xpos[site_id][2]


def simulate(initial_velocity):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.00)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    set_joint_velocity(model, data, "gantry_pitch", initial_velocity)

    mujoco.mj_forward(model, data)

    min_z = foot_world_z(model, data)
    touchdown_time = None

    while data.time < 1.0:
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

        z = foot_world_z(model, data)
        min_z = min(min_z, z)

        if touchdown_time is None and foot_in_contact(model, data):
            touchdown_time = data.time

    return min_z, touchdown_time


def main():
    velocities = [-1.00, -0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75, 1.00]

    print("gantry_pitch_vel | min_foot_z | touchdown_time")
    print("-----------------------------------------------")

    for velocity in velocities:
        min_z, touchdown_time = simulate(velocity)

        if touchdown_time is None:
            touchdown_text = "none"
        else:
            touchdown_text = f"{touchdown_time:.3f} s"

        print(f"{velocity:>16.2f} | {min_z:>10.4f} | {touchdown_text}")


if __name__ == "__main__":
    main()