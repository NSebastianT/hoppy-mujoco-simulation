from pathlib import Path
import re

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


def make_model_xml(hip_motor_mass):
    xml = MODEL_PATH.read_text(encoding="utf-8")

    pattern = r'(<geom name="hip_motor_mass"[^>]*mass=")[^"]+(")'
    replacement = rf'\g<1>{hip_motor_mass}\2'

    xml_new, count = re.subn(pattern, replacement, xml, flags=re.DOTALL)

    if count != 1:
        raise RuntimeError("Could not replace hip_motor_mass. Check models/hoppy.xml.")

    return xml_new


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


def foot_world_z(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    return data.site_xpos[site_id][2]


def simulate_mass(hip_motor_mass):
    xml = make_model_xml(hip_motor_mass)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.50)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    set_joint_velocity(model, data, "gantry_pitch", 1.0)

    mujoco.mj_forward(model, data)

    sim_time = 2.0
    contact_steps = 0
    total_steps = 0
    transitions = 0

    previous_contact = foot_in_contact(model, data)
    min_foot_z = foot_world_z(model, data)
    max_foot_z = foot_world_z(model, data)

    while data.time < sim_time:
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

        contact = foot_in_contact(model, data)

        if contact:
            contact_steps += 1

        if contact != previous_contact:
            transitions += 1
            previous_contact = contact

        z = foot_world_z(model, data)
        min_foot_z = min(min_foot_z, z)
        max_foot_z = max(max_foot_z, z)

        total_steps += 1

    contact_fraction = contact_steps / total_steps
    final_pitch = get_joint_qpos(model, data, "gantry_pitch")

    return {
        "mass": hip_motor_mass,
        "contact_fraction": contact_fraction,
        "transitions": transitions,
        "min_foot_z": min_foot_z,
        "max_foot_z": max_foot_z,
        "final_pitch": final_pitch,
    }


def main():
    masses = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2]

    print("mass_kg | contact_% | transitions | min_z_m | max_z_m | final_pitch_rad")
    print("-----------------------------------------------------------------------")

    for mass in masses:
        result = simulate_mass(mass)

        print(
            f"{result['mass']:>7.2f} | "
            f"{100.0 * result['contact_fraction']:>9.2f} | "
            f"{result['transitions']:>11d} | "
            f"{result['min_foot_z']:>7.4f} | "
            f"{result['max_foot_z']:>7.4f} | "
            f"{result['final_pitch']:>15.4f}"
        )


if __name__ == "__main__":
    main()