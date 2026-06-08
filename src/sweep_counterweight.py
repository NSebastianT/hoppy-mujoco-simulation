from pathlib import Path
import re

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


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


def foot_world_z(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    return data.site_xpos[site_id][2]


def simulate(counterweight_mass):
    xml = make_model_xml(counterweight_mass)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.00)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    set_joint_velocity(model, data, "gantry_pitch", 0.50)

    mujoco.mj_forward(model, data)

    sim_time = 2.0

    touchdown_count = 0
    liftoff_count = 0
    contact_steps = 0
    total_steps = 0

    previous_contact = foot_in_contact(model, data)

    min_foot_z = foot_world_z(model, data)
    max_foot_z = foot_world_z(model, data)

    min_pitch = get_joint_qpos(model, data, "gantry_pitch")
    max_pitch = get_joint_qpos(model, data, "gantry_pitch")

    first_touchdown = None
    first_liftoff = None

    while data.time < sim_time:
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)

        contact = foot_in_contact(model, data)

        if contact:
            contact_steps += 1

        if contact and not previous_contact:
            touchdown_count += 1
            if first_touchdown is None:
                first_touchdown = data.time

        if previous_contact and not contact:
            liftoff_count += 1
            if first_liftoff is None:
                first_liftoff = data.time

        previous_contact = contact

        z = foot_world_z(model, data)
        pitch = get_joint_qpos(model, data, "gantry_pitch")

        min_foot_z = min(min_foot_z, z)
        max_foot_z = max(max_foot_z, z)

        min_pitch = min(min_pitch, pitch)
        max_pitch = max(max_pitch, pitch)

        total_steps += 1

    contact_fraction = contact_steps / total_steps
    final_pitch = get_joint_qpos(model, data, "gantry_pitch")

    return {
        "mass": counterweight_mass,
        "touchdowns": touchdown_count,
        "liftoffs": liftoff_count,
        "first_touchdown": first_touchdown,
        "first_liftoff": first_liftoff,
        "contact_fraction": contact_fraction,
        "min_foot_z": min_foot_z,
        "max_foot_z": max_foot_z,
        "min_pitch": min_pitch,
        "max_pitch": max_pitch,
        "final_pitch": final_pitch,
    }


def format_time(value):
    if value is None:
        return "none"
    return f"{value:.3f}"


def main():
    masses = [0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.3]

    print("cw_kg | TD | LO | first_TD | first_LO | contact_% | max_z | pitch_min | pitch_max | final_pitch")
    print("---------------------------------------------------------------------------------------------")

    for mass in masses:
        r = simulate(mass)

        print(
            f"{r['mass']:>5.2f} | "
            f"{r['touchdowns']:>2d} | "
            f"{r['liftoffs']:>2d} | "
            f"{format_time(r['first_touchdown']):>8} | "
            f"{format_time(r['first_liftoff']):>8} | "
            f"{100.0 * r['contact_fraction']:>9.2f} | "
            f"{r['max_foot_z']:>5.3f} | "
            f"{r['min_pitch']:>9.3f} | "
            f"{r['max_pitch']:>9.3f} | "
            f"{r['final_pitch']:>11.3f}"
        )


if __name__ == "__main__":
    main()