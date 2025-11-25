# FILDX — Fielding Training Device Simulator Documentation

Version: 1.0

Last updated: 2025-11-25

---

**Purpose**

FILDX (Fielding Device Simulator) is a simulator application designed to model, visualize, and tune a cricket fielding training device. It helps developers, coaches, and hardware engineers understand device behavior, evaluate control strategies, and create training scenarios by simulating ball trajectories, collisions with surfaces, fielding mechanisms, and interactive UI feedback.

**Intended audience**: developers, simulation engineers, physics/controls engineers, and cricket coaches.

---

**Contents**

- Project goals
- High-level architecture
- Physics and mathematics (models & equations)
- Physics engine design (integration, collisions, contact models)
- Implementation details (data structures, algorithms, parameters)
- UI/UX and visualization
- Input/output, data logging and telemetry
- Testing, validation and calibration
- Performance, optimization and profiling guidance
- Future work and extension points
- References and appendix (equations)

---

**Project Goals**

- Provide a reproducible, configurable simulator of a cricket fielding training device that models ball flight, spin, bounces, catcher/target surfaces, and simple mechanical release mechanisms.
- Allow parameter tuning (initial velocity, spin, release angle, device timings, surface coefficients) to match hardware.
- Offer real-time visualization with telemetry overlays for trajectory, velocity, spin, energies, and predicted landing zones.
- Provide an extendable physics engine suitable for adding more complex interactions (multi-ball, moving targets, robotic hands).

---

**High-level Architecture**

- Frontend (UI): `index.html` plus JS/CSS (visualization, controls, telemetry, scenario editor).
- Simulation core (physics engine): responsible for stepping the world forward, integrating rigid-body and point-mass dynamics of the ball, computing aerodynamic forces, collision detection and response, and event generation (bounce events, catch events).
- Scenario/config layer: JSON-based scenario definitions describing initial conditions, device parameters, surface models, and camera/visualization preferences.
- Logger: structured CSV/JSON output of simulation runs, including per-frame state and derived diagnostics.

Common data flow: UI -> scenario JSON -> simulation core -> per-step state -> renderer/UI overlays -> logger.

---

**Physics & Mathematics**

The simulator treats the ball as a point mass with orientation (for spin) and uses 3D kinematics with gravity, aerodynamic drag, Magnus (lift due to spin), and contact forces on bounce.

Assumptions:
- Ball is a rigid sphere with mass $m$ and radius $r$.
- Air is homogeneous and stationary; wind can be added later as a uniform vector field.
- Ground and device contact surfaces are planar or simple convex shapes.

State representation per ball:

- Position: $\\mathbf{x}(t) = (x, y, z)$
- Velocity: $\\mathbf{v}(t)$
- Angular velocity (spin): $\\boldsymbol{\\omega}(t)$ (vector)

Forces considered:

- Gravity: $\\mathbf{F}_g = m \\mathbf{g}$ where $\\mathbf{g} = (0,0,-g)$ and $g \\approx 9.81\\ \\text{m/s}^2$.
- Aerodynamic drag: $\\mathbf{F}_d = -\\tfrac{1}{2} \\rho A C_d ||\\mathbf{v}_\\text{rel}|| \\mathbf{v}_\\text{rel}$.
- Magnus lift: $\\mathbf{F}_m = \\tfrac{1}{2} \\rho A C_l (\\boldsymbol{\\hat{\\omega}} \\times \\mathbf{v}_\\text{rel}) ||\\mathbf{v}_\\text{rel}||$ (or more detailed dependance $C_l = f(\\text{spin parameter})$).

Where:
- $\\rho$ is air density (1.225 kg/m^3 at sea level),
- $A = \\pi r^2$ is projected area,
- $C_d$ is drag coefficient (function of Reynolds number; approximate constant 0.3–0.5 for a cricket ball depending on seam/roughness),
- $C_l$ is lift coefficient depending on spin rate and Reynolds number.

Spin parameter (nondimensional): $S = \\dfrac{r \\|\\boldsymbol{\\omega}\\|}{\\|\\mathbf{v}\\|}$.

Equations of motion (ODE):

$$m \\dfrac{d\\mathbf{v}}{dt} = m\\mathbf{g} + \\mathbf{F}_d + \\mathbf{F}_m$$
$$\\dfrac{d\\mathbf{x}}{dt} = \\mathbf{v}$$

Angular state update (simple model): we may optionally model spin decay using an exponential damping:

$$\\dfrac{d\\boldsymbol{\\omega}}{dt} = -k_\\omega \\boldsymbol{\\omega}$$

with $k_\\omega$ a small damping constant.

Collision/bounce model uses impulse-based response (coefficient of restitution $e$) and friction (tangential impulse) to transfer linear/angular momentum during contact.

Bounce math (idealized instantaneous collision):

Let the normal at contact be $\\mathbf{n}$ (pointing out of surface). Decompose pre-impact velocity at contact point into normal and tangential components and compute post-impact impulses.

Normal impulse magnitude $J_n$ enforces relative normal velocity $v_n^+$ satisfying $v_n^+ = -e\\, v_n^-$. 

Tangential impulse $\\mathbf{J}_t$ limited by Coulomb friction: $\\|\\mathbf{J}_t\\| \\leq \\mu J_n$ where $\\mu$ is friction coefficient.

These impulses update linear velocity and angular velocity accordingly. For a ball (mass $m$, moment of inertia $I = \\frac{2}{5} m r^2$ for a solid sphere; for cricket ball this is a hollow sphere approximation) the impulse updates are standard rigid-body impulse formulas.

---

**Physics Engine Design**

Goals: stable, deterministic, easy to tune, accurate enough for training device matching.

Key choices:

- Time stepping: fixed time step integrator (recommended) for determinism. Typical step size: $\\Delta t = 1/200\\ \\text{s}$ to $1/100\\ \\text{s}$ depending on desired fidelity and performance.
- Integrator: semi-implicit (symplectic) Euler or a second-order integrator (RK2) for translational dynamics. Semi-implicit Euler is fast and stable for this usage.
- Collision detection: simple analytic tests for sphere-plane and sphere-convex shapes (device surfaces, floors). Use continuous collision detection (CCD) for high-speed passes: compute time of impact inside time step by solving sphere-plane intersection along motion.
- Collision response: impulse-based with normal restitution and tangential friction. Resolve penetrations by projecting out along the normal and applying a velocity-level impulse.
- Sub-stepping: if time of impact occurs inside step, split the step: integrate to TOI, apply impulse, then integrate remaining time.

Main simulation loop (pseudocode):

```
for each frame:
  t = 0
  while t < frame_dt:
    dt = min(fixed_dt, frame_dt - t)
    perform CCD to find earliest impact in dt (if any)
    if impact at toi < dt:
      integrate dynamics for toi
      apply collision response at toi
      dt -= toi
      t += toi
    else:
      integrate dynamics for dt
      t += dt
  render state
  log telemetry
```

Integrator details (semi-implicit Euler):

1. Compute forces at current state: $\\mathbf{F}(t)$
2. Update velocity: $\\mathbf{v}_{t+\\Delta t} = \\mathbf{v}_t + \\Delta t \, \\mathbf{F}(t) / m$
3. Update position: $\\mathbf{x}_{t+\\Delta t} = \\mathbf{x}_t + \\Delta t \, \\mathbf{v}_{t+\\Delta t}$

This treats drag/Magnus using current velocity (explicit), but remains stable if dt is sufficiently small.

Collision impulse computation (sphere-plane simplified):

1. Relative normal velocity at contact: $v_n = \\mathbf{v} \\cdot \\mathbf{n}$.
2. Desired post-collision normal velocity: $v_n^+ = -e v_n^-$.
3. Impulse magnitude: $J_n = -(1+e) v_n / (1/m + 0)$ (sphere plane: plane has infinite mass), so $J_n = -(1+e) m v_n$.
4. Tangential impulse computed from relative tangential velocity and friction threshold.

For more precise response involving ball spin, compute relative velocity at contact point including rotational component: $\\mathbf{v}_c = \\mathbf{v} + \\boldsymbol{\\omega} \\times (-r \\mathbf{n})$.

Update angular velocity using $\\Delta \\boldsymbol{\\omega} = I^{-1} (\\mathbf{r} \\times \\mathbf{J})$.

---

**Implementation Details**

Data structures:

- Ball object: contains mass `m`, radius `r`, position `x`, velocity `v`, angular velocity `omega`, coefficients `Cd`, `Cl`, `e`, `mu`.
- Surface object: type ('plane', 'convex'), transform, normal, friction and restitution properties.
- Scenario: JSON file with device parameters, ball initial conditions, surface properties, camera and visualization settings.
- Telemetry entry per step: timestamp, position, velocity, spin, forces, energy, event labels.

Parameter defaults (suggested starting values):

- Ball mass `m` = 0.156 kg (approx cricket ball), radius `r` = 0.036 m (approx 72 mm diameter),
- Drag coefficient `Cd` = 0.35 (tweak for seam/roughness),
- Magnus coefficient scaling `Cl_scale` = 0.2 (tune),
- Restitution `e` = 0.45–0.6 depending on pitch surface,
- Friction `mu` = 0.4–0.6.

Time-step & determinism:

- Use fixed-step simulation and deterministic random seeds for reproducible runs. Convert variable framerate to fixed substeps so simulation output is independent of renderer FPS.

Logging and reproducibility:

- Save scenario config and RNG seed with every run. Log per-step state as CSV/JSON for offline analysis and comparison to hardware recordings.

---

**UI / Visualization**

Goals: Give coaches and developers clear, real-time feedback on ball behavior with adjustable parameters.

Suggested UI components:

- Top bar: scenario select, run/pause/step controls, playback speed.
- Left panel: parameter sliders (initial speed, elevation angle, azimuth, spin magnitude and axis, seam orientation, ball mass, Cd, Cl, restitution, friction).
- Right panel: telemetry & diagnostics (current speed, spin rpm, predicted landing zone, bounce count, energy).
- Main viewport: 3D (or 2D overhead + side) visualization of field, device, ball trajectory, trail, predicted trajectory overlay, and impact markers.
- Graph overlays: height vs. time, speed vs. time, spin vs. time.

Interactions:

- Click to set release point or aim direction.
- Click-and-drag sliders to tune device parameters live.
- Export run data as JSON/CSV.

Accessibility & UX:

- Color blind friendly palettes for overlays.
- Keyboard shortcuts for common actions (space=play/pause, left/right step).

---

**Inputs, Outputs & Formats**

- Scenario file (JSON) format example:

```json
{
  "ball": {"m":0.156, "r":0.036, "Cd":0.35, "Cl_scale":0.2},
  "release": {"speed":20.0, "elevation_deg":30, "azimuth_deg":0, "spin_rpm":1500, "spin_axis": [0,1,0]},
  "surface": {"type":"plane","restitution":0.55,"friction":0.45},
  "device": {"type":"launcher","position":[0,0,1.0]}
}
```

- Logger output: CSV with columns [time, x,y,z, vx,vy,vz, omega_x,omega_y,omega_z, events].

---

**Testing, Validation & Calibration**

Unit tests and validation suites should cover:

- Kinematic integrator tests: constant acceleration should match analytic solutions.
- Drag/Magnus tests: compare simulated steady-state drag deceleration against expected values.
- Collision tests: known-impact scenarios (drop from height onto plane) check bounce height vs expected from restitution.
- Energy checks: monitor expected energy dissipation from drag and inelastic collisions.

Calibration strategy to match hardware:

1. Record high-speed camera data and ball telemetry from the physical launcher for a set of standardized shots.
2. Fit drag coefficient `Cd` and Magnus scaling `Cl_scale` by minimizing position/time residuals between simulated trajectory and recorded data (least-squares optimization).
3. Fit restitution `e` and friction `mu` using bounce heights and tangential velocities after bounce.

Suggested tests:

- Regression tests for trajectories with fixed seed.
- Numerical stability tests with varying dt.

---

**Performance & Optimization**

Typical bottlenecks: collision detection (if many objects), rendering, and per-step computation of nonlinear aerodynamic forces.

Optimizations:

- Use analytical sphere-plane and sphere-convex collision tests rather than mesh queries.
- Cache derived constants (projected area, inv mass) per ball.
- Only simulate detailed aerodynamic forces at sufficiently large speeds; below a threshold approximate with linear drag.
- Use spatial partitioning if many objects are present (quadtree/k-d tree) — likely not necessary for a small device.

Parallelism:

- Multi-ball scenarios can update each ball independently in parallel until collisions between balls become significant.
- Offload heavy logging to a separate thread or batch writes.

---

**Extensibility & Integration Points**

- Add wind as a vector field and gust models.
- Replace the point-mass ball with a full rigid-body model to simulate seam orientation and complex spin interactions.
- Integrate with hardware via a simple REST or WebSocket API to run scenarios and stream telemetry.
- Add machine learning modules to predict optimal device parameters for targeted landing zones.

---

**Appendix: Key Equations and Constants**

- Gravity: $g = 9.81\\ \mathrm{m/s^2}$
- Air density: $\\rho = 1.225\\ \mathrm{kg/m^3}$ (sea level)
- Projected area: $A = \\pi r^2$.
- Drag: $\\mathbf{F}_d = -\\tfrac{1}{2}\\rho A C_d ||\\mathbf{v}|| \\mathbf{v}$.
- Magnus (simple): $\\mathbf{F}_m \\propto (\\boldsymbol{\\omega} \\times \\mathbf{v})$ with scale depending on $C_l$.

Moment of inertia of solid sphere: $I = \\tfrac{2}{5} m r^2$ (use for angular impulse updates).

---

**Quick start: files & commands**

- Place scenario JSON files in `scenarios/`.
- Open `index.html` in a browser to run interactive visualizer.

If the repository includes a local server (dev environment), start it and open the UI; if not, open `index.html` directly in a modern browser.

---

**Future Work / Roadmap**

1. Add hardware-in-the-loop connectivity (WebSocket API + calibration assistant).
2. Full rigid-body seam-aware ball model and advanced aerodynamic models dependent on seam orientation.
3. Multi-ball scheduling, replay & training session editor, and performance reports for players.
4. Move simulator core into a standalone WASM module for faster physics and reuse across platforms.

---

**References**

- Bird, D., & Biondini, M. (2014). Ball aerodynamics and the Magnus effect overview.
- Cross, R. (2015). Sports ball dynamics and impact modelling.
- Standard rigid-body dynamics and collision response literature (e.g., Baraff, M.)


