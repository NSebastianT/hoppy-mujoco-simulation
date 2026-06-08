from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = model.jnt_qposadr[joint_id]
    data.qpos[qpos_id] = value


def foot_world_z(model, data):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "foot_site")
    return data.site_xpos[site_id][2]


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    gantry_pitch = 0.00
    hip_values = [0.00, 0.20, 0.35, 0.50, 0.70]
    knee_values = [-1.20, -0.90, -0.70, -0.50, -0.20, 0.00]

    print("gantry_pitch = 0.00 rad")
    print("hip   | knee  | foot_world_z")
    print("-----------------------------")

    for hip in hip_values:
        for knee in knee_values:
            data = mujoco.MjData(model)

            set_joint(model, data, "gantry_pitch", gantry_pitch)
            set_joint(model, data, "hip", hip)
            set_joint(model, data, "knee", knee)

            mujoco.mj_forward(model, data)

            print(f"{hip:>5.2f} | {knee:>5.2f} | {foot_world_z(model, data):>12.4f} m")

        print()
        

if __name__ == "__main__":
    main()