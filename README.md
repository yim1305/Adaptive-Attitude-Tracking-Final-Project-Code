# Adaptive Attitude Tracking with a Four-CMG Pyramid

MATLAB simulation for my EML 6350 (Nonlinear Control) final project at the University of Florida. A rigid spacecraft with unknown principal moments of inertia tracks a desired attitude using an adaptive controller, and a pseudoinverse steering law maps the commanded torque to gimbal rates for a four-CMG pyramid.

## Running

Open MATLAB in `code/` and run `main.m` (R2020a or newer, no toolboxes needed). It runs the eight test cases, prints the result tables in the command window, and saves the figures as PDF and PNG. It takes a few minutes because of the tight ode45 tolerances.

## Folders

- `code/`
  - `main.m`: parameters, gains, test cases, and result tables
  - `attitude_dynamics.m`: closed-loop equations of motion
  - `controller.m`: torque command and adaptive update law
  - `steering_algorithm.m`: pseudoinverse gimbal-rate command
  - `cmg_model.m`: CMG array momentum and Jacobian
  - `reference_motion.m`: slew and periodic reference motions
  - `simulation_results.m`: tracking errors and actuator checks for each run
  - `plot_results.m`: full-width figures
  - `plot_column_figures.m`: single-column figures
  - `axis_angle_quat.m`, `quaternion_kinematics.m`, `quat2rotm_body.m`, `skew.m`: quaternion and matrix helpers
- `figures/full/`: full page width figures (7.16 in)
- `figures/column/`: single-column figures (3.5 in)
