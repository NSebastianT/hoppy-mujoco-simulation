"""Interactive viewer of the CAD hopping, for a live demo.

Opens the MuJoCo passive viewer and runs the hopping controller in real time.

On Linux with the Intel Xe GPU the GLFW viewer can crash; force the NVIDIA GPU:
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
        .venv-py312/bin/python src/view_cad_hop.py
On Windows/macOS just run it normally.
"""
import os
import time

# Use the windowed backend (not the offscreen EGL one used for rendering).
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco
import mujoco.viewer

from cad_hop_controller import build_model_and_data, Hopper


def main():
    model, data = build_model_and_data()
    hopper = Hopper(model)
    hopper.reset(data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [-0.15, 0.0, 0.20]
        viewer.cam.distance = 2.7
        viewer.cam.azimuth = 110
        viewer.cam.elevation = -28
        viewer.sync()

        while viewer.is_running():
            start = time.time()
            data.ctrl[:] = hopper.control(data)
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
