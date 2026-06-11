"""Render a 360-degree spin of the faithful CAD model (official URDF assembly).

Uses models/hoppy_cad_view.xml, built from the official HOPPY-E0-final URDF
with the official (decimated) meshes. Headless and cross-platform.

Usage:
    python src/render_cad_view.py [out.mp4]
"""
import sys
from pathlib import Path

from render_utils import select_gl_backend, write_video

select_gl_backend()

import mujoco  # noqa: E402  (must come after select_gl_backend)

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "hoppy_cad_view.xml"
WIDTH, HEIGHT, FPS = 640, 480, 30


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "renders" / "cad_view.mp4"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [-0.35, 0.0, 0.18]
    cam.distance = 1.8
    cam.elevation = -12

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
