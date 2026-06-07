from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
    data.qpos[qpos_id] = value


def foot_height(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    return data.site_xpos[site_id][2]


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    pitches = [-0.20, -0.10, 0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.55]

    print("gantry_pitch | foot_world_z")
    print("---------------------------")

    for pitch in pitches:
        data = mujoco.MjData(model)

        set_joint(model, data, "gantry_pitch", pitch)
        set_joint(model, data, "hip", 0.35)
        set_joint(model, data, "knee", -0.70)

        mujoco.mj_forward(model, data)

        print(f"{pitch:>12.2f} | {foot_height(model, data):>12.4f} m")


if __name__ == "__main__":
    main()