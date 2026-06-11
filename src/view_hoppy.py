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


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)  # "home" standing pose (foot on floor)

    mujoco.mj_forward(model, data)

    print("HOPPY model loaded correctly.")
    print(f"nq: {model.nq}")
    print(f"nv: {model.nv}")
    print(f"nu: {model.nu}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < 10:
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    print("HOPPY viewer test finished.")


if __name__ == "__main__":
    main()