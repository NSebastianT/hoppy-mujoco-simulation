"""Render the robot with the official CAD meshes opaque (primitives hidden).

Produces a 360-degree spin MP4 so the official HOPPY CAD form can be inspected
without the interactive viewer. Headless and cross-platform.

Usage:
    python src/render_cad_view.py [out.mp4]
"""
import sys
from pathlib import Path

from render_utils import select_gl_backend, write_video

select_gl_backend()

import mujoco  # noqa: E402  (must come after select_gl_backend)

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy.xml"
WIDTH, HEIGHT, FPS = 640, 480, 30

CAD_GEOMS = ["official_gantry_visual", "official_upper_leg_visual",
             "official_lower_leg_visual"]
HIDE_GEOMS = ["yaw_hub", "gantry_bar", "counterweight", "hip_motor_mass",
              "upper_leg", "lower_leg",
              "reference_link1_visual", "reference_link2_visual",
              "reference_link3_visual", "reference_link4_visual"]


def gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "renders" / "cad_view.mp4"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    for n in CAD_GEOMS:
        model.geom_rgba[gid(model, n)][3] = 1.0
    for n in HIDE_GEOMS:
        model.geom_rgba[gid(model, n)][3] = 0.0

    mujoco.mj_resetDataKeyframe(model, data, 0)  # "home" standing pose (foot on floor)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.35, 0.0, 0.30]
    cam.distance = 1.4
    cam.elevation = -18

    frames = []
    n_frames = 120
    for i in range(n_frames):
        cam.azimuth = 360.0 * i / n_frames
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render().copy())

    renderer.close()
    out = write_video(frames, out_path, fps=FPS)
    print(f"Wrote CAD spin ({len(frames)} frames) -> {out}")


if __name__ == "__main__":
    main()
