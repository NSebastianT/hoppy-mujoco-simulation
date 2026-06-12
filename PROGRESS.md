# HOPPY project log (MuJoCo)

Team coordination document. Summarizes where we started, what is done, what is
left and the phase plan, mapped to the rubric. Updated every time we make
progress, so it is clear what to say in the presentation and what comes next.

Last update: June 12, 2026.

## Starting point

- `main` had the hopping simulation with the hybrid controller working on a
  simplified capsule model (`models/hoppy.xml`), with 3 DoF (pitch, hip, knee).
- The official CAD was placed by hand on top of the model and did not assemble
  correctly.
- Along the way some rubric items had been lost (knee spring, recommended
  solver config) and the actuator values were not justified.

## Status per phase (rubric, 100 points)

Legend: [x] done, [~] partial, [ ] pending.

### Phase 1 - Mechanical model (20 pts) -- complete
- [x] 4 DoF: yaw + pitch passive, hip + knee active.
- [x] Recommended solver: RK4, Newton, timestep 1 ms (1 kHz), iterations 50,
      tolerance 1e-8.
- [x] Knee parallel spring (stiffness 1.0, springref 0; lowered from 2.0 to
      calm the in-flight bounce without losing its energy-storage role).
- [x] Armature = N^2*Ir on hip and knee, with the real motor values.
- [x] Equivalent back-EMF damping = (kv*kt/Rw)*N^2.
- [x] Counterweight on the opposite end of the gantry.

### Phase 2 - Actuator constraints (10 pts) -- complete
- [x] Motor torque saturation applied in the model and the controller:
      +-12.2 N*m (hip) and +-13 N*m (knee), from the 30 A driver peak
      (0.405 N*m/A at the output). It used to be +-35, unrealistic.
- [x] Comparison simulations (tools/compare_actuator.py -> plot
      results/plots/cad_comparison_height.png). Runs the same hop toggling
      each effect and measures the body height peak:
        baseline (all realistic) ....... 0.404 m
        no armature .................... 0.519 m  (no rotor inertia, jumps higher)
        no damping ..................... 0.442 m  (no back-EMF losses, higher)
        no knee spring ................. 0.394 m  (WITHOUT the spring it jumps LESS)
        no saturation (+-35 Nm) ........ 0.620 m  (more torque, higher)
      Analysis: armature and saturation are the dominant limits on the hop.
      Removing the spring LOWERS the peak: the parallel spring does store and
      release energy in the cycle (exactly what rubric 1.4 asks for). This is
      the validation of the actuator model.

### Phase 3 - Foot-ground contact (15 pts) -- complete
- [x] Hard contact configured and justified (foot + floor):
        solref = "0.01 1"  -> 10 ms time constant, damping ratio 1
                              (critically damped): fast contact without
                              artificial bouncing.
        solimp = "0.95 0.99 0.001" -> high impedance (stiff), little compliance.
        friction = "1.5 0.02 0.001" -> high tangential friction -> minimal
                              unwanted slipping.
- [x] Contact quality measured (6 s of hopping): max penetration of the
      collision sphere ~18 mm (it is invisible; the visible leg mesh stays
      >0.11 m above the floor, never looks sunk); low foot slip in contact
      (~0.18 m/s mean, almost all of it the intentional tangential travel of
      the laps, not slip); smooth normal force (peak ~256 N) without
      multi-bounce landings. Hardening solref/solimp further does not reduce
      the penetration (it is the small sphere at impact, not compliance).
- [x] Touchdown/liftoff criterion: touchdown = foot (or leg capsule)-floor
      geom pair present in the MuJoCo contacts, with a minimum flight time
      guard (MIN_FLIGHT) against chattering; liftoff = contact lost after
      MIN_STANCE, or the MAX_STANCE cap. Robust to contact bounces thanks to
      the minimum time guards.

### Phase 4 - State machine and hybrid control (40 pts) -- complete
- [x] 1 kHz loop and contact-based FLIGHT/STANCE machine.
- [x] REAL Cartesian control in flight (rubric formula 4.3):
      tau = J^T * Kp * (pd - p) - Kd * qdot_estimated, with pd = the foot
      position of the reference pose at the current gantry attitude (FK). A
      weak joint posture PD is added because hip and knee move the foot in
      nearly the same direction at the flight pose (J^T J close to singular)
      and J^T alone leaves one joint direction without stiffness. Flight used
      to be pure joint-space PD; now it matches what the rubric asks for.
- [x] Force control in stance (GRF via transposed Jacobian), with a tangential
      horizontal component to travel around the pillar. The GRF push is now
      applied ONLY during real contact (gating): before, the Bezier profile
      kept "pushing" in the air because of the time guards (real contact lasts
      ~25 ms but MIN_STANCE holds the state 120 ms), which whipped the leg and
      the inelastic impacts rectified it into backwards rotation.
- [x] Hop energy solved: sustained hops with a steady peak of ~0.40 m and
      ~0.6 s flights (tools/eval_cad_hop.py).
- [x] Bezier force profile in stance, 10 ms alpha blend and a stronger leg PD
      on the CAD.
- [x] Continuous yaw rotation achieved: removed the range="-3.14 3.14" stop on
      joint1 that came from the URDF export. The "stalling at half a turn" was
      the robot hitting that limit and bouncing back, NOT impact physics. The
      official MATLAB simulator has no yaw position limit (the real HOPPY laps
      its base). With the stop gone the robot does full laps (+372 degrees in
      15 s, yaw monotonicity 1.000).

### Phase 5 - Sensors and signal processing (15 pts) -- complete
- [x] goBilda 5202 encoder emulation: joint3/joint4 quantized to 751.8
      counts/rev at the output shaft.
- [x] Velocity estimated by filtered numerical derivative; the CAD PD no
      longer uses raw `qvel` for velocity feedback.
- [x] CAD logging to `results/logs/cad_states.csv` and regenerable plots in
      `results/plots/` with positions, velocities, contact, torques and the
      hybrid state.

## Motor data (for the report)

goBilda 5202 Series Yellow Jacket, 26.9:1 ratio (part 5202-2402-0027), RS-555
base, Pololu VNH5019 driver at 12 V (team BOM).

- Free speed 223 RPM, free current 0.25 A.
- Stall torque 3.73 N*m, stall current 9.2 A.
- Rw = V/Istall = 12/9.2 = 1.30 ohm.
- OFFICIAL NOMINAL VALUES (from HOPPY-Project/Simulator_MATLAB/get_params.m
  and "List of nominal parameters.pdf", no longer estimates):
  kt = 0.0135 N*m/A, kv = 0.0186 V*s/rad, Ir = 7e-6 kg*m^2.
- N_hip = 26.9, N_knee = 28.8. V_max = 12 V, I_max = 30 A (driver peak).
- armature = N^2*Ir -> 0.00507 (hip), 0.00581 (knee).
- damping = (kv*kt/Rw)*N^2 -> 0.140 (hip), 0.160 (knee).
- Peak torque = kt*N*I_max -> ~11-12 N*m; we saturate at +-12.2/13 N*m.

## Models and how to run

Always use the `.venv-py312` environment (it has mujoco, imageio and trimesh).

- `models/hoppy_cad_physics.xml`: official CAD with REAL physics (foot
  collision, actuators, joint dynamics). This is the deliverable and what the
  simulation runs: the real model hopping.
- `models/hoppy_cad_view.xml`: faithful official CAD assembled from the URDF.
  Visual only.
- `models/hoppy.xml`: simplified physics model (capsules), kept for
  development and quick tuning.

## Plan for the final goal (CAD hopping with physics)

The professor wants to see the real model hopping. Steps:
1. [x] CAD model with physics (`hoppy_cad_physics.xml`): foot/floor collision,
       hip/knee motors, armature/damping/spring. Falls and lands stably.
2. [x] Drop test (valid contact, nothing blows up).
3. [x] Hybrid controller ported (`src/cad_hop_controller.py`): the CAD HOPS
       with physics (foot rises ~0.2 m, boom pitches, stable). This is the
       version to show the professor: the real model hopping.
4. [x] CAD hop tuning. The floating counterweight blocks were removed and
       folded into the Link2 inertia (mass 3.87654 kg, COM x=0.06075 m),
       leaving the joint2 bias around -1.2 N*m without floating blocks (the
       visible weights are now two capsules and a clamp drawn at the boom
       end). The CAD controller uses a 0.15 s Bezier force profile, a 10 ms
       alpha blend and a strong leg PD. The foot collision was aligned with
       the visual tip of Link4 and a thin lower-leg capsule was added. Stance
       includes a 1.0 s warmup that ramps the Bezier push to avoid the initial
       lunge; the posture PD runs at full strength from t=0 (the old 210 N
       settle force was removed: it added nothing to the CW startup and it
       cranked the no-cw hip). In 6 s it reports 8 real flights (>0.10 s,
       ~0.6 s each), first_hop_peak ~0.369 m, steady_peak ~0.401 m,
       yaw_progress ~2.41 rad, yaw monotonicity 1.000, mesh_min_z ~0.112 m,
       stable and inside the torque limits. The foot remains the dominant
       contact over the leg capsule.
5. [x] Sensors and plots (Phase 5): the CAD controller uses velocity estimated
       from the quantized encoder; `src/run_cad_logged.py` generates the CSV
       and the rubric plots in `results/`.
6. [x] Hard contact tuned (Phase 3, see the phase section above).
7. [x] Final render of the CAD hopping with physics in
       `results/renders/cad_hopping.mp4` (not versioned).

Commands:

    .venv-py312/bin/python src/cad_hop_controller.py    # CAD hopping with physics -> results/renders/cad_hopping.mp4
    .venv-py312/bin/python src/cad_hop_controller.py --no-cw  # WITHOUT counterweight (it sinks) -> cad_hopping_nocw.mp4
    .venv-py312/bin/python src/render_simulation.py     # capsule model hopping -> results/renders/hopping.mp4
    .venv-py312/bin/python src/render_cad_view.py       # static CAD (spin) -> results/renders/cad_view.mp4
    .venv-py312/bin/python src/run_logged_simulation.py # simplified model logs and plots in results/
    .venv-py312/bin/python src/run_cad_logged.py        # CAD CSV and plots -> results/logs and results/plots

LIVE demo of the CAD hopping (interactive MuJoCo window):

    # Windows / macOS:
    .venv-py312/bin/python src/view_cad_hop.py

    # Linux (force the NVIDIA GPU; the viewer crashes on Intel Xe):
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia .venv-py312/bin/python src/view_cad_hop.py

The interactive viewer opens a GLFW window. On Linux it crashes on the Intel
Xe GL, so on Linux the NVIDIA GPU is forced (PRIME offload) with the variables
above; tested and working. For recording mp4 use the offscreen renders.

## Branches

- `develop`: integration. `main`: delivery.
- `rubric-physics`: physics phases (current working branch).
- `cad-in-action`: kinematic CAD animation (preview, not merged).

## Known limitations

- Knee bounce in flight: practically SOLVED on the counterweight model (the
  one we present). With the Cartesian flight PD the in-air knee oscillation is
  ~0.5 degrees peak to peak (imperceptible). History: the real parallel spring
  (pulls to springref=0) fought the old joint-space PD; the stiffness was
  lowered from 2.0 to 1.0 and the Cartesian control finished calming it. The
  --no-cw variant keeps a brief knee twitch in its short hops (spring +
  encoder), inherent to that variant.
- Laps: SOLVED. The "stalling at half a turn" was the range="-3.14 3.14" stop
  on the yaw joint inherited from the URDF export: the robot hit the limit at
  pi and bounced back. The real gantry has no stop (neither does the official
  MATLAB simulator), so it was removed and the robot now does full continuous
  laps. The render/viewer camera was raised (elevation -28) so the leg can be
  seen going around instead of hiding behind the base.
- The --no-cw variant (no counterweight) advances slowly and in small jerks,
  with yaw swings of up to ~5 degrees. That is the expected behavior of the
  leg-heavy boom: hops are tiny (~0.1 s of air), the state machine cycle runs
  at the edge of the real contact, and the inelastic impacts erase the angular
  momentum on every landing. Exactly the point of the demo: without the
  counterweight the system cannot hop well.

## Open decisions

- No technical items pending. Ir is the official nominal value (7e-6,
  get_params.m) and the continuous laps are solved (it was the yaw joint
  stop). What remains is the report and the presentation.
