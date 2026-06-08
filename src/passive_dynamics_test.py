import time
from pathlib import Path

import mujoco
import mujoco.viewer


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


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.00)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)

    set_joint_velocity(model, data, "gantry_pitch", 0.50)

    mujoco.mj_forward(model, data)

    print("Passive dynamics test")
    print("No controller. Motors set to zero.")
    print(f"Initial foot height: {foot_world_z(model, data):.4f} m")
    print(f"Initial contact: {foot_in_contact(model, data)}")

    previous_contact = foot_in_contact(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.25, 0.0, 0.30]
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        viewer.sync()

        input("Viewer ready. Press Enter to start passive simulation...")

        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < 20:
            data.ctrl[:] = 0.0

            mujoco.mj_step(model, data)

            contact = foot_in_contact(model, data)

            if contact and not previous_contact:
                print(f"Touchdown at t = {data.time:.3f} s")

            if previous_contact and not contact:
                print(f"Lift-off at t = {data.time:.3f} s")

            previous_contact = contact

            viewer.sync()
            time.sleep(0.003)

    print("Passive dynamics test finished.")


if __name__ == "__main__":
    main()