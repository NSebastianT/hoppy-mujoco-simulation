import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
    data.qpos[qpos_id] = value


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


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    set_joint(model, data, "gantry_pitch", 0.55)
    set_joint(model, data, "hip", 0.35)
    set_joint(model, data, "knee", -0.70)
    mujoco.mj_forward(model, data)

    previous_contact = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < 8:
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)

            contact = foot_in_contact(model, data)

            if contact and not previous_contact:
                print(f"Touchdown detected at t = {data.time:.3f} s")

            if previous_contact and not contact:
                print(f"Lift-off detected at t = {data.time:.3f} s")

            previous_contact = contact

            viewer.sync()
            time.sleep(model.opt.timestep)

    print("Contact test finished.")


if __name__ == "__main__":
    main()