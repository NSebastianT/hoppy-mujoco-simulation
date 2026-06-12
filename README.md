# HOPPY MuJoCo Simulation

This repository contains a MuJoCo simulation of HOPPY, a dynamic single-legged robot designed to hop around a fixed gantry.

The project currently has two working directions:

1. A simplified dynamic MuJoCo model that can hop.
2. A CAD-based visual version that uses official HOPPY STEP files exported with FreeCAD.

The current simulation is not presented as a fully exact replica of the physical HOPPY robot. The dynamics are simplified on purpose so the model can be controlled, tested, logged, and explained. The official CAD geometry is used mainly to improve visual fidelity.

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

## Branches

`main`

Final working branch. This branch contains the current version of the project: simplified MuJoCo dynamics, hybrid hopping control, and official HOPPY CAD visual meshes integrated into the simulation.

`repeat-hop-control`

Experimental branch with an adaptive controller for repeated hopping. This branch produced the best repeated hopping behavior, but it has not been merged into the CAD visual version yet.

`cad-official-hoppy`

Development branch used to integrate the official HOPPY CAD meshes with FreeCAD. After merging, its relevant changes are included in `main`.

## Current model

The MuJoCo model currently uses:

- one passive gantry pitch joint,
- one actuated hip joint,
- one actuated knee joint,
- a counterweight,
- a foot contact point,
- simple collision geometry,
- official CAD meshes as visual geometry.

The current dynamic model has three generalized coordinates:

```text
gantry_pitch
hip
knee
```

The official HOPPY CAD model has more mechanical detail than the current dynamic model. For now, the full linkage is not rebuilt as a complete multi-joint mechanism. Instead, the simulation uses simplified bodies for dynamics and official CAD meshes for visualization.

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

The simplified physics model `models/hoppy.xml` (capsules + the tuned
controller) is kept separate and is what the simulation actually runs. An
earlier attempt that overlaid hand-placed FreeCAD meshes on that physics model
was removed in favor of the faithful URDF-based view above.

## Main files

```text
models/hoppy.xml
src/hybrid_controller_test.py
src/run_logged_simulation.py
```

`models/hoppy.xml` defines the MuJoCo model.

`src/hybrid_controller_test.py` runs the interactive hybrid controller test.

`src/run_logged_simulation.py` generates logs and plots for analysis.

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

During `FLIGHT`, the controller uses a joint-space PD controller to keep the leg in a reasonable posture before landing.

During `STANCE`, the controller computes the foot Jacobian and applies a desired vertical foot force using:

```text
tau = J^T F
```

This generates a vertical push during ground contact.

The actuator commands are saturated using a torque limit:

```text
TORQUE_LIMIT = 35.0
```

## What works

The current project already has:

- a MuJoCo model that loads correctly,
- a working floor contact,
- touchdown detection,
- lift-off detection,
- a `FLIGHT / STANCE` hybrid state machine,
- torque saturation,
- a Jacobian-based stance push,
- a simplified model that can hop,
- official HOPPY CAD visual meshes loaded into MuJoCo,
- an official-looking gantry visual integrated into the dynamic scene,
- generated plots from logged simulations.

## Plots already generated

The project generated plots for:

```text
joint_positions.png
estimated_velocities.png
foot_position.png
foot_world_z.png
hip_height.png
normal_force.png
hybrid_state.png
torques.png
gantry_pitch.png
gantry_pitch_velocity.png
foot_relative_position.png
foot_vertical_velocity.png
```

These plots were used to analyze:

- joint motion,
- estimated velocity,
- foot height,
- hip height,
- contact timing,
- normal force,
- hybrid state transitions,
- torque commands,
- gantry pitch motion.

The plots should be regenerated from the final selected branch before delivery, because the controller and visual model changed during development.

## Current limitations

The current simulation is still simplified.

Important limitations:

- The visual CAD geometry is not used for contact.
- Contact still uses simple MuJoCo collision geometry.
- The model does not yet use CAD-derived inertias.
- The full official HOPPY linkage has not been rebuilt as a complete multi-joint mechanism.
- The current dynamic model only uses gantry pitch, hip, and knee.
- The CAD visual offsets still need tuning.
- The adaptive repeated hopping controller is in `repeat-hop-control`, not merged into `cad-official-hoppy`.
- The current `cad-official-hoppy` branch prioritizes visual integration over final control tuning.

## Pending work

These are the remaining tasks.

### 1. Decide the final delivery branch

Choose which branch will be used for the final demo:

```text
main
repeat-hop-control
cad-official-hoppy
```

Recommended path:

```text
cad-official-hoppy
```

because it includes the official CAD visual model.

Then decide whether to merge the adaptive hopping controller from:

```text
repeat-hop-control
```

into:

```text
cad-official-hoppy
```

### 2. Regenerate all plots

Run the logged simulation again from the final branch.

Expected output:

```text
results/logs/hybrid_log.csv
results/plots/*.png
```

The plots should include at least:

```text
joint positions
joint/foot estimated velocities
foot height
hip height
normal force
hybrid state
torques
gantry pitch
gantry pitch velocity
```

These plots are needed for the report and presentation.

### 3. Tune the CAD visual alignment

The CAD meshes are loaded and visible, but some offsets still need adjustment.

Things to check:

```text
Link2 gantry alignment
Link3 upper-leg alignment
Link4 lower-leg/foot alignment
hip area visual overlap
counterweight visual position
```

This is visual tuning only. It should not change the collision geometry unless needed.

### 4. Decide whether to keep the static CAD reference

The scene currently can include a static official CAD reference model.

For the final demo, decide whether to:

```text
keep it as a comparison reference
```

or

```text
remove it to keep the scene clean
```

### 5. Decide what to do with the black simplified hip box

The black box is useful dynamically because it represents mass near the hip.

For the final visual version, decide whether to:

```text
keep it opaque
make it transparent
reduce its size
hide it visually but keep an equivalent inertial body
```

The safest option is to keep the simple mass for dynamics and make it less visually dominant.

### 6. Confirm model scope with the professor

Ask whether the expected final model is:

```text
a simplified MuJoCo dynamic model with official CAD visuals
```

or

```text
a full CAD/URDF-style reconstruction of the official HOPPY mechanism
```

This matters because the current model is not a full reconstruction of every official mechanical joint.

### 7. Document simplifications clearly

The report should clearly state:

```text
The model uses simplified collision geometry.
The CAD meshes are visual only.
The controller is tested on the simplified dynamic model.
The official CAD improves visual fidelity but does not define contact dynamics.
```

### 8. Compare controller versions

Compare at least two versions:

```text
stable Jacobian controller
adaptive repeated hopping controller
```

Explain which one is more stable and which one produces better hopping.

### 9. Verify torque saturation

The controller clips torque commands.

Before delivery, check plots or terminal output to confirm:

```text
maximum hip torque
maximum knee torque
whether either actuator saturates
```

### 10. Prepare final explanation

The final explanation should cover:

```text
model structure
contact detection
FLIGHT / STANCE state machine
Jacobian stance control
torque saturation
logged plots
CAD visual integration
limitations
next steps
```

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
A MuJoCo simulation of a simplified HOPPY-like hopper with official HOPPY CAD visual integration.
```

The strongest technical points are:

```text
hybrid FLIGHT/STANCE control
contact-based transitions
Jacobian-based stance push
torque saturation
logged plots
official CAD visual integration
```

The main limitation to state clearly is:

```text
The CAD geometry improves the visual model, but the physical simulation still uses simplified dynamics and collision geometry.
```