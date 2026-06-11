"""Shared helpers for headless (offscreen) rendering.

`select_gl_backend()` must run BEFORE `import mujoco`, so call it at the very
top of any render script.
"""
import os
import sys
from pathlib import Path


def select_gl_backend():
    """Pick an OpenGL backend that works headless, cross-platform.

    Respects an existing MUJOCO_GL. On Linux it forces EGL, because the default
    GLFW path crashes on some Intel Xe / hybrid-GPU drivers. On Windows and
    macOS it leaves MuJoCo's default (WGL / CGL), which works fine.
    """
    if os.environ.get("MUJOCO_GL"):
        return
    if sys.platform.startswith("linux"):
        os.environ["MUJOCO_GL"] = "egl"


def write_video(frames, path, fps=30):
    """Encode RGB frames (HxWx3 uint8 arrays) to an MP4.

    Uses imageio + the bundled imageio-ffmpeg binary, so no system ffmpeg
    install is required (important for Windows teammates).
    """
    import imageio.v2 as imageio

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(str(path), fps=fps, codec="libx264",
                            quality=8, macro_block_size=16) as writer:
        for frame in frames:
            writer.append_data(frame)
    return path
