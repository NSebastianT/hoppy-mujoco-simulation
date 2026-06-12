# HOPPY MuJoCo Simulation

This repository contains a MuJoCo simulation of HOPPY, a dynamic single-legged robot designed to hop around a fixed gantry.

The deliverable is the official HOPPY CAD model hopping with real physics: the
URDF assembly with the nominal motor parameters (reflected rotor inertia,
back-EMF damping, torque limits), the knee parallel spring, hard foot-ground
contact, and a hybrid FLIGHT/STANCE controller. A simplified capsule model is
kept alongside for development. Collision geometry is intentionally simple (a
foot sphere and a leg capsule aligned with the meshes); everything else comes
from the official HOPPY data.

## Code overview (key files)

Models:
- `models/hoppy_cad_physics.xml` - official CAD (from the HOPPY-E0 URDF) turned into
  a physics model: 4 DoF (yaw+pitch passive, hip+knee active), foot collision,
  hip/knee motors with torque limits, reflected-inertia armature, back-EMF damping,
  knee parallel spring, and a counterweight folded into the boom inertia. This is the
  model used for the presentation (the real robot hopping with physics).
- `models/hoppy_cad_view.xml` - faithful CAD assembly from the URDF, visual only.
- `models/hoppy.xml` - simplified capsule model used to develop and tune the controller.

Control and tools:
- `src/cad_hop_controller.py` - hybrid FLIGHT/STANCE hopping controller for the CAD
  model: transposed-Jacobian Cartesian PD in flight, desired ground-reaction force
  with a Bezier profile in stance (vertical = hop, tangential = travel), smooth
  state transition and soft startup, torque saturation, and encoder-emulated joint
  velocity (no raw qvel). Renders the hop to an MP4.
- `src/view_cad_hop.py` - interactive live viewer (for the demo).
- `src/run_cad_logged.py` - logs all states to a CSV and generates the rubric plots.
- `tools/eval_cad_hop.py` - headless metrics (hop quality, contact, torque, yaw).
- `tools/compare_actuator.py` - Phase 2 comparison of armature/damping/spring/saturation.
- `PROGRESO.md` - living progress and coordination log (phase-by-phase status).

Motor (goBilda 5202 26.9:1, VNH5019 driver at 12 V) sets the physical parameters:
torque limit ~12-13 N*m (30 A peak), armature = N^2*Ir, damping = (kv*kt/Rw)*N^2.

## Installation

Prerequisites: Python 3.10 or newer, and git.

Clone and enter the repo:

```
git clone https://github.com/NSebastianT/hoppy-mujoco-simulation.git
cd hoppy-mujoco-simulation
```

Create the environment (named `.venv-py312` so the run commands below work as-is)
and install the dependencies:

Linux / macOS:

```
python3 -m venv .venv-py312
.venv-py312/bin/python -m pip install -r requirements.txt
```

Windows (PowerShell):

```
py -m venv .venv-py312
.venv-py312\Scripts\python -m pip install -r requirements.txt
```

Notes:
- `requirements.txt` pulls MuJoCo, numpy, matplotlib, scipy, imageio (+ ffmpeg, so
  no system ffmpeg is needed) and trimesh/fast-simplification (only for regenerating
  the CAD meshes).
- On Windows/macOS the run commands are the same but use
  `.venv-py312\Scripts\python` instead of `.venv-py312/bin/python`.
- The live MuJoCo viewer opens a GLFW window. On Windows/macOS it works directly;
  on Linux with an Intel Xe GPU it crashes, so force the NVIDIA GPU (see "How to
  run each part"). Offscreen renders (the `render_*` / mp4 scripts) are unaffected.

See "How to run each part" below for usage.

## Branches

`main`

Delivery branch (tagged releases v1.0.x). Contains the official CAD model
hopping with real physics and the hybrid controller.

`develop`

Integration branch. Work lands here before going to `main`.

`rubric-physics`

Current working branch for the physics/rubric phases.

`cad-official-hoppy`, `repeat-hop-control`, `cad-in-action`

Older development branches (CAD mesh integration, early controller experiments,
kinematic CAD animation). Their relevant changes are already in `main`.

## Current model

The deliverable is `models/hoppy_cad_physics.xml`: the official CAD assembly
(URDF meshes and joint transforms) turned into a physics model. It has the four
HOPPY degrees of freedom:

```text
yaw      (gantry, passive, continuous - no hard stop, like the real rig)
pitch    (gantry, passive)
hip      (actuated)
knee     (actuated)
```

plus the counterweight on the boom, reflected rotor inertia (`armature`),
equivalent back-EMF damping, the knee parallel spring, torque-limited motors,
and foot/lower-leg collision geometry against the floor. The CAD meshes are
visual only; contact uses a foot sphere and a thin leg capsule aligned with the
mesh.

`models/hoppy.xml` is the earlier simplified capsule model, kept for
development and quick tuning.

## Official CAD integration

The official CAD comes from the RoboDesignLab HOPPY project, delivered as the
`HOPPY-E0-final` SolidWorks-to-URDF export (a URDF plus STL meshes).

`models/hoppy_cad_view.xml` is a faithful, **visual-only** assembly built
straight from that URDF: the exact joint transforms plus the official meshes,
so the robot assembles correctly. Render it with `src/render_cad_view.py`.

The official `Link2`/`Link3` STLs exceed MuJoCo's 200k-face limit, so they are
decimated (preserving each mesh's coordinate frame) into:

```text
assets/meshes/hoppy_official_urdf/
```

To regenerate them from the official export:

```text
python tools/prepare_cad_view_meshes.py path/to/HOPPY-E0-final/meshes
```

The same URDF data is the basis of `models/hoppy_cad_physics.xml`, which adds
the physics on top (collision, actuators, joint dynamics) and is what the
simulation runs. An earlier attempt that overlaid hand-placed FreeCAD meshes on
the capsule model was removed in favor of the faithful URDF-based assembly.

## Main files

```text
models/hoppy_cad_physics.xml
src/cad_hop_controller.py
src/view_cad_hop.py
src/run_cad_logged.py
tools/eval_cad_hop.py
tools/compare_actuator.py
```

`models/hoppy_cad_physics.xml` defines the model that simulates.

`src/cad_hop_controller.py` has the hybrid controller (`Hopper`) and renders
the hopping to mp4; `src/view_cad_hop.py` runs the same controller in the
interactive viewer. Both accept `--no-cw` for the no-counterweight variant.

`src/run_cad_logged.py` generates the CSV and the rubric plots;
`tools/eval_cad_hop.py` prints headless metrics and `tools/compare_actuator.py`
runs the phase 2 comparison sims.

## Controller

The controller uses a hybrid state machine with two states:

```text
FLIGHT
STANCE
```

The code detects foot-ground contact using MuJoCo contacts between:

```text
foot
floor
```

During `FLIGHT`, the controller runs a Cartesian PD on the foot through the
transposed Jacobian (`tau = J^T Kp (pd - p) - Kd qdot`, with the velocity
estimated from the emulated encoder), plus a weak joint posture term, to keep
the leg in a landing posture.

During `STANCE`, the controller computes the foot Jacobian and applies a desired ground-reaction force, while the foot is actually in contact, using:

```text
tau = J^T F
```

The vertical Bezier profile produces the jump and the tangential component drives the robot around the gantry.

The actuator commands are saturated at the real motor torque limits:

```text
TORQUE_LIMIT = [12.2, 13.0]  # N*m, hip / knee
```

## What works

- The official CAD model hops with real physics: steady ~0.40 m peaks, ~0.6 s
  flights, torques inside the motor limits.
- It laps the gantry continuously (the yaw advances monotonically, full turns).
- Touchdown/lift-off detection from MuJoCo contacts, with the ground-reaction
  push applied only while the foot is actually in contact.
- The `--no-cw` variant shows the counterweight's role: leg-heavy boom, the
  robot sinks and can barely hop.
- Encoder-emulated velocity (quantized position + filtered derivative) drives
  the controller; raw `qvel` is not used for feedback.
- CSV logging and all rubric plots regenerate from `src/run_cad_logged.py`.

## Plots already generated

`src/run_cad_logged.py` writes `results/logs/cad_states.csv` and these plots to
`results/plots/`:

```text
cad_joint_positions.png      joint positions (hip, knee, gantry)
cad_joint_velocities.png     true vs encoder-estimated joint velocities
cad_foot_position.png        Cartesian foot position
cad_foot_velocity.png        true vs estimated Cartesian foot velocity
cad_contact_force.png        normal contact force
cad_torques.png              motor torques vs the saturation limits
cad_hybrid_state.png         FLIGHT/STANCE state over time
```

`tools/compare_actuator.py` adds `cad_comparison_height.png` (phase 2). The
`results/` folder is gitignored, so regenerate before the presentation.

## Current limitations

- The `--no-cw` variant advances slowly and in small jerks, with a brief knee
  twitch each cycle (the real parallel spring plus the encoder-estimated
  velocity, in a regime of 0.1 s hops). That is the leg-heavy dynamics it is
  meant to demonstrate: tiny hops, and the inelastic landings erase the
  angular momentum each cycle. In the counterweight model the knee oscillation
  in flight is negligible (~0.5 deg peak to peak).
- The CAD meshes are visual only; contact uses a foot sphere and a leg capsule
  aligned with the mesh (hard contact, ~2.7 mm steady penetration of the
  invisible sphere; the visible mesh never touches the floor).

## Pending work

- Write the report and presentation. The material is ready: this README,
  `PROGRESO.md`, the plots in `results/plots/`, the CSV log and the rendered
  videos.
- Regenerate `results/` from the delivery branch before presenting.

## How to run each part

Use the `.venv-py312` environment and run from the repo root. Generated outputs
go to `results/` (gitignored). On Linux the interactive viewer must be forced
onto the NVIDIA GPU (the Intel Xe GL crashes it); on Windows/macOS run it plainly.

Deliverable - the official CAD hopping with real physics:

```
# live, interactive (Linux):
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia .venv-py312/bin/python src/view_cad_hop.py
.venv-py312/bin/python src/view_cad_hop.py --no-cw     # version without the counterweight

# render to mp4 (results/renders/):
.venv-py312/bin/python src/cad_hop_controller.py          # with counterweight  -> cad_hopping.mp4
.venv-py312/bin/python src/cad_hop_controller.py --no-cw  # without (it sinks)  -> cad_hopping_nocw.mp4
```

Per rubric phase:

```
# Phase 5 - sensors: CSV + plots (encoder velocity, contact force, torques, state):
.venv-py312/bin/python src/run_cad_logged.py        # -> results/logs/cad_states.csv, results/plots/cad_*.png

# Phase 2 - actuator: comparison of armature/damping/spring/saturation:
.venv-py312/bin/python tools/compare_actuator.py    # -> results/plots/cad_comparison_height.png

# metrics (hop quality, contact, torque, yaw):
.venv-py312/bin/python tools/eval_cad_hop.py
```

Visual-only CAD and the simplified development model:

```
.venv-py312/bin/python src/render_cad_view.py       # CAD 360 spin (no physics)
.venv-py312/bin/python src/render_simulation.py     # simplified capsule model hopping
.venv-py312/bin/python src/run_logged_simulation.py # simplified model logs + plots
```

## Headless rendering (videos, cross-platform)

The interactive viewer (`view_hoppy.py`, `hybrid_controller_test.py`) opens a
GLFW window. That works on Windows and macOS, but crashes on some Linux Intel
Xe / hybrid-GPU drivers. The render scripts below sidestep that: they render
**offscreen** to an MP4, so they work on any OS and produce a shareable video.

They auto-pick the GL backend (EGL on Linux, default on Windows/macOS) and use
the bundled `imageio-ffmpeg`, so **no system ffmpeg install is needed**.

Render the hopping simulation:

```bat
python src\render_simulation.py
```

Render a 360 spin of the faithful CAD model (`models/hoppy_cad_view.xml`):

```bat
python src\render_cad_view.py
```

Both write to `results/renders/` (gitignored). Optional args:
`python src\render_simulation.py out.mp4 8` (output path, seconds).

## Faithful CAD model (`models/hoppy_cad_view.xml`)

`models/hoppy.xml` is the simplified **physics** model (capsules + the tuned
controller). Its CAD overlay was placed by hand and is only approximate.

`models/hoppy_cad_view.xml` is a **visual-only** model built straight from the
official `HOPPY-E0-final` URDF export (SolidWorks). It uses the exact joint
transforms and the official meshes, so the robot assembles correctly. It is not
used for simulation — the foot rests ~4 cm above the floor because the real rig
is boom-mounted (the leg hops above the surface).

The official `Link2`/`Link3` STLs exceed MuJoCo's 200k-face limit, so they are
decimated into `assets/meshes/hoppy_official_urdf/`. To regenerate from the
official export:

```bat
python tools\prepare_cad_view_meshes.py path\to\HOPPY-E0-final\meshes
```

## How to regenerate CAD meshes

FreeCAD is used to export the official STEP assemblies to simplified STL meshes.

Example:

```bat
"C:\Users\nabor\AppData\Local\Programs\FreeCAD 1.1\bin\freecadcmd.exe" -c "import sys, runpy; sys.argv=['export_step_meshes_freecad.py', r'external\HOPPY-Project\CAD\Final_Assembly\Link4\Link4_Assembly.STEP', r'assets\meshes\hoppy_official\link4_visual.stl']; runpy.run_path(r'tools\export_step_meshes_freecad.py')"
```

The mesh simplification settings are inside:

```text
tools/export_step_meshes_freecad.py
```

## Notes for the report

The project should be presented as:

```text
A MuJoCo simulation of the official HOPPY robot (CAD model) hopping with real
physics and a hybrid controller, using the official nominal motor parameters.
```

The strongest technical points are:

```text
official nominal motor parameters (armature, damping, torque limits)
hard foot-ground contact with touchdown/lift-off detection
hybrid FLIGHT/STANCE control with contact-based transitions
Cartesian transposed-Jacobian PD in flight, Bezier GRF profile in stance
encoder-emulated velocity estimation (no raw qvel feedback)
continuous laps around the gantry
counterweight vs no-counterweight comparison
```

The main simplification to state clearly is:

```text
Contact uses simple collision geometry (foot sphere + leg capsule) aligned
with the CAD meshes; the meshes themselves are visual only. Masses, inertias
and motor constants come from the official HOPPY nominal parameters.
```