import time
import mujoco
import mujoco.viewer


MODEL_PATH = "models/smoke_test.xml"


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    print("Model loaded correctly.")
    print(f"Number of generalized coordinates: {model.nq}")
    print(f"Number of generalized velocities: {model.nv}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and time.time() - start_time < 5:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    print("Simulation finished correctly.")


if __name__ == "__main__":
    main()