# HOPPY MuJoCo Simulation

MuJoCo simulation of [HOPPY](https://github.com/RoboDesignLab/HOPPY-Project), a
one-legged hopping robot mounted on a boom that pivots around a fixed base.

The official HOPPY CAD model hops with real physics: nominal motor parameters
(reflected rotor inertia, back-EMF damping, torque limits), the knee parallel
spring, hard foot-ground contact, and a hybrid FLIGHT/STANCE controller. The
system is underactuated: 4 DoF (boom yaw and pitch are passive, hip and knee
are driven) with only 2 motors.

Results in short: steady hops of ~0.40 m with ~0.6 s flights, torques inside
the real motor limits, and continuous laps around the base. A second variant
(`--no-cw`) removes the counterweight to show why it matters: the boom becomes
leg-heavy and the robot can barely lift off.

## Repository layout

Models (`models/`):

- `hoppy_cad_physics.xml` - the model that simulates. Official CAD assembly
  (HOPPY-E0 URDF meshes and joint transforms) plus the physics: foot and
  lower-leg collision, hip/knee motors with torque limits, `armature` =
  N^2*Ir, back-EMF `damping`, knee parallel spring, and the counterweight on
  the boom. Continuous yaw (no hard stop, like the real rig).
- `hoppy_cad_view.xml` - faithful CAD assembly from the URDF, visual only.
- `hoppy.xml` - simplified capsule model used to develop and tune the
  controller, kept for reference.

Code:

- `src/cad_hop_controller.py` - the hybrid controller (`Hopper`) and the mp4
  render of the hop. Cartesian transposed-Jacobian PD in flight, Bezier
  ground-reaction force profile in stance (vertical = hop, tangential =
  travel), contact-gated push, soft startup, torque saturation, and
  encoder-emulated joint velocity (no raw `qvel`).
- `src/view_cad_hop.py` - the same controller in the interactive viewer, for
  the live demo. Both scripts accept `--no-cw`.
- `src/run_cad_logged.py` - logs every state to a CSV and generates the plots.
- `tools/eval_cad_hop.py` - headless metrics (hop quality, contact, torque,
  yaw progress).
- `tools/compare_actuator.py` - comparison sims toggling
  armature/damping/spring/saturation.
- `PROGRESO.md` - living progress log, phase by phase.

Motor: goBilda 5202 26.9:1 (RS-555 base) with a VNH5019 driver at 12 V. The
official nominal constants (kt = 0.0135, kv = 0.0186, Rw = 1.30, Ir = 7e-6)
give armature = N^2*Ir = 0.00507/0.00581, damping = (kv*kt/Rw)*N^2 =
0.140/0.160, and a torque limit of 12.2/13 N*m (hip/knee).

## Installation

Prerequisites: Python 3.10 or newer, and git.

Clone and enter the repo:

```
git clone https://github.com/NSebastianT/hoppy-mujoco-simulation.git
cd hoppy-mujoco-simulation
```

Create the environment (named `.venv-py312` so the run commands below work
as-is) and install the dependencies:

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

- `requirements.txt` pulls MuJoCo, numpy, matplotlib, scipy, imageio (with a
  bundled ffmpeg, so no system ffmpeg is needed) and
  trimesh/fast-simplification (only used to regenerate the CAD meshes).
- On Windows use `.venv-py312\Scripts\python` wherever the commands below say
  `.venv-py312/bin/python`.

## How to run

Run from the repo root. Generated outputs go to `results/` (gitignored).

Live demo (interactive MuJoCo window):

```
# Windows / macOS:
.venv-py312/bin/python src/view_cad_hop.py

# Linux (force the NVIDIA GPU; the viewer crashes on the Intel Xe GL):
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia .venv-py312/bin/python src/view_cad_hop.py

# either OS, without the counterweight:
... src/view_cad_hop.py --no-cw
```

Videos (offscreen render, works the same on any OS):

```
.venv-py312/bin/python src/cad_hop_controller.py          # -> results/renders/cad_hopping.mp4
.venv-py312/bin/python src/cad_hop_controller.py --no-cw  # -> results/renders/cad_hopping_nocw.mp4
.venv-py312/bin/python src/render_cad_view.py             # CAD 360 spin, no physics
```

Plots, logs and metrics:

```
.venv-py312/bin/python src/run_cad_logged.py        # CSV + plots -> results/logs, results/plots
.venv-py312/bin/python tools/compare_actuator.py    # actuator comparison -> cad_comparison_height.png
.venv-py312/bin/python tools/eval_cad_hop.py        # headless metrics, printed
```

Simplified development model (optional):

```
.venv-py312/bin/python src/render_simulation.py     # capsule model hopping -> hopping.mp4
.venv-py312/bin/python src/run_logged_simulation.py # capsule model logs + plots
```

## The model

The official CAD comes from the RoboDesignLab HOPPY project, delivered as the
`HOPPY-E0-final` SolidWorks-to-URDF export (a URDF plus STL meshes).
`hoppy_cad_physics.xml` keeps the exact joint transforms and meshes and adds
the physics on top. The four degrees of freedom:

```text
yaw      boom, passive, continuous (no hard stop, like the real rig)
pitch    boom, passive
hip      actuated
knee     actuated
```

The CAD meshes are visual only. Contact uses a foot sphere and a thin
lower-leg capsule aligned with the mesh, against a plane floor with hard
contact settings (`solref="0.01 1"`, `solimp="0.95 0.99 0.001"`, friction
1.5). The solver setup follows the recommended configuration: RK4, Newton,
1 ms timestep, 50 iterations, tolerance 1e-8.

The counterweight (two weights clamped near the boom end) balances the
leg-heavy boom; its mass is folded into the boom inertia and drawn as
geometry. Running with `--no-cw` swaps in the no-counterweight inertia, which
is the comparison that shows its effect on the dynamics.

The official `Link2`/`Link3` STLs exceed MuJoCo's 200k-face limit, so the
meshes in `assets/meshes/hoppy_official_urdf/` are decimated copies
(preserving each mesh's coordinate frame). To regenerate them from the
official export:

```
.venv-py312/bin/python tools/prepare_cad_view_meshes.py path/to/HOPPY-E0-final/meshes
```

(`tools/export_step_meshes_freecad.py` exists for exporting STL from the
original STEP assemblies; it runs inside FreeCAD's `freecadcmd` and is only
needed if you start from the raw CAD instead of the URDF export.)

## The controller

Hybrid state machine with two states, `FLIGHT` and `STANCE`. Transitions come
from contact detection (the foot-floor or leg-floor geom pair appearing in the
MuJoCo contacts), debounced with minimum flight/stance time guards. The
control loop runs at 1 kHz, same as the physics timestep.

During `FLIGHT`, a Cartesian PD on the foot through the transposed Jacobian
keeps the leg in a landing posture:

```text
tau = J^T * Kp * (pd - p) - Kd * qdot_est
```

where `pd` is the foot position of the reference pose at the current boom
attitude, and `qdot_est` is the encoder-estimated velocity. A weak joint-space
posture term is added because at the flight pose the hip and knee move the
foot in nearly the same direction (J^T J close to singular), so J^T alone
leaves one joint direction without stiffness.

During `STANCE`, and only while the foot is actually in contact, a desired
ground-reaction force is applied through the same Jacobian:

```text
tau = J^T * F
```

with a Bezier profile on the vertical component (the jump) and a constant
tangential component (the travel around the base). Transitions blend smoothly
over 10 ms and the first second ramps the push up to avoid a startup lunge.

Velocity feedback never uses raw `qvel`: joint positions are quantized to the
real encoder resolution (751.8 counts/rev at the output) and velocity is
estimated with a filtered numerical derivative.

All torque commands saturate at the real motor limits, 12.2 N*m (hip) and
13 N*m (knee).

## Plots and logs

`src/run_cad_logged.py` writes `results/logs/cad_states.csv` and these plots
to `results/plots/`:

```text
cad_joint_positions.png      joint positions (yaw, pitch, hip, knee)
cad_joint_velocities.png     true vs encoder-estimated joint velocities
cad_foot_position.png        Cartesian foot position
cad_foot_velocity.png        true vs estimated Cartesian foot velocity
cad_contact_force.png        normal contact force
cad_torques.png              motor torques vs the saturation limits
cad_hybrid_state.png         FLIGHT/STANCE state over time
```

`tools/compare_actuator.py` adds `cad_comparison_height.png`: the same hop
with armature, damping, the knee spring and the torque saturation toggled one
at a time. Armature and saturation are the dominant limits; removing the
spring lowers the hop, which shows the parallel spring really stores and
releases energy in the cycle.

`results/` is gitignored - regenerate before presenting.

## Known limitations

- Contact geometry is intentionally simple (foot sphere + leg capsule); the
  CAD meshes do not collide. Steady-state penetration of the invisible foot
  sphere is ~2.7 mm and the visible mesh never touches the floor.
- The `--no-cw` variant advances slowly and in small jerks, with a brief knee
  twitch each cycle (real spring + encoder velocity in a regime of 0.1 s
  hops). That is the leg-heavy dynamics it exists to demonstrate. On the
  counterweight model the in-flight knee oscillation is negligible (~0.5 deg
  peak to peak).

## Branches

- `main` - delivery branch (tagged releases v1.0.x).
- `develop` - integration; work lands here before `main`.
- `rubric-physics` - current working branch for the physics phases.
- `cad-official-hoppy`, `repeat-hop-control`, `cad-in-action` - older
  development branches, already merged where relevant.
