# SCARA Robotic Arm Simulator | MATLAB

![MATLAB](https://img.shields.io/badge/MATLAB-R2018b%2B-orange)
![Toolboxes](https://img.shields.io/badge/toolboxes-none-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)

This is an interactive MATLAB simulator for a 4-DOF **SCARA (RRPR)** robotic arm. It implements **forward kinematics** and **workspace analysis** using **Denavit–Hartenberg (DH) kinematic modelling**. You control the joint angles and the link lengths live.

<!--
  Add a screenshot after your first run: use any screenshot tool, or in
  MATLAB R2020b+ run  fig = scara_simulator;  exportapp(fig, 'docs/images/simulator.png')
  then uncomment the next line:

![SCARA simulator](docs/images/simulator.png)
-->

---

## Features

**3D Simulation tab**
- Sliders and numeric fields for all four joints: θ₁, θ₂ (deg), d₃ (mm), θ₄ (deg).
- Live link-length controls for a₁, a₂, column height d₁, tool offset d₄ and quill stroke.
- A shaded 3-D model of the arm (base, links, quill, tool and gripper) that updates in real time.
- DH coordinate frames {0}–{4} drawn on the arm. A transparent "x-ray" mode is also available.
- A live **DH parameter table** and the **base-to-tool transform ⁰T₄ = A₁A₂A₃A₄**.
- Tool pose read-out (X, Y, Z, yaw φ), **manipulability** w = a₁a₂|sin θ₂|, and a singularity indicator.
- Tool-tip trail, Home and Random-pose motions, and a continuous **Animate** mode.

**Workspace Analysis tab**
- Joint-space sampling mapped through the kinematics. The top view (XY) and the 3-D reachable volume are coloured by manipulability or by elbow configuration.
- Envelope curves (joint-limit edges and the singular arc θ₂ = 0), with r_min / r_max circles.
- Metrics: minimum and maximum reach, **planar area** (exact polar integral), **workspace volume**, Z range, stroke, and mean manipulability.
- Adjustable θ₁ / θ₂ limits and sampling density. The current tool position is shown live on the top view.

**Kinematics library** (`src/`)
- DH link transform, DH table, forward kinematics, closed-form inverse kinematics, geometric Jacobian, reach, workspace area (two independent methods) and plotting.
- 16 unit tests, including DH product vs. closed form, IK round trip, Jacobian vs. finite differences, and area vs. analytical cases.

---

## Quick start

```matlab
>> cd scara-robotic-arm-simulator
>> scara_simulator
```

The simulator adds `src/` to the MATLAB path automatically.

Example scripts:

```matlab
>> run examples/demo_forward_kinematics.m   % DH table, A1..A4, 0T4, closed-form check
>> run examples/demo_workspace_analysis.m   % reach/area/volume + link-length design study
>> run examples/demo_ik_trajectory.m        % circular pick path with inverse kinematics
```

Using the library directly:

```matlab
addpath src
robot = scara_params('a1', 0.40, 'a2', 0.30);        % metres / radians
q     = [pi/6, pi/4, 0.05, 0];                       % [theta1 theta2 d3 theta4]

[T, frames, P, pose] = scara_fk(q, robot);           % 0T4, all frames, origins, [x y z phi]
[qIK, ok]            = scara_ik(pose, robot, 'auto');% inverse kinematics
[J, w]               = scara_jacobian(q, robot);     % 6x4 Jacobian, manipulability

ws = scara_workspace(robot, 150);                    % sampled workspace + metrics
scara_plot_workspace(ws, robot);
fprintf('Area = %.4f m^2, volume = %.4f m^3\n', ws.area, ws.volume);
```

---

## Kinematic model

Standard DH convention, $A_i = \mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i)\,\mathrm{Trans}_x(a_i)\,\mathrm{Rot}_x(\alpha_i)$:

| Link | θᵢ | dᵢ | aᵢ | αᵢ |
|:----:|:--:|:--:|:--:|:--:|
| 1 | θ₁\* | d₁ | a₁ | 0 |
| 2 | θ₂\* | 0 | a₂ | π |
| 3 | 0 | d₃\* | 0 | 0 |
| 4 | θ₄\* | d₄ | 0 | 0 |

The DH product gives the tool position in closed form:

$$
x = a_1\cos\theta_1 + a_2\cos(\theta_1+\theta_2),\qquad
y = a_1\sin\theta_1 + a_2\sin(\theta_1+\theta_2),\qquad
z = d_1 - d_3 - d_4
$$

The tool yaw is φ = θ₁ + θ₂ − θ₄.

The full derivation is in **[docs/kinematics.md](docs/kinematics.md)**. It covers the link matrices, ⁰T₄, IK, the Jacobian, singularities and the workspace-area integral.

Default robot (`scara_params`): a₁ = 350 mm, a₂ = 250 mm, d₁ = 400 mm, d₄ = 50 mm, θ₁ ∈ ±150°, θ₂ ∈ ±145°, d₃ ∈ [0, 200] mm, θ₄ ∈ ±180°. With these values the reach is 204–600 mm, the planar area is about 0.977 m² and the volume is about 0.195 m³.

---

## Project structure

```
scara-robotic-arm-simulator/
├── scara_simulator.m            # Interactive GUI (entry point)
├── src/
│   ├── scara_params.m           # Robot geometry, joint limits, validation
│   ├── dh_transform.m           # Standard DH link transform A_i
│   ├── scara_dh_table.m         # DH parameter table for a joint vector
│   ├── scara_fk.m               # Forward kinematics 0T4 = A1 A2 A3 A4
│   ├── scara_ik.m               # Closed-form inverse kinematics
│   ├── scara_jacobian.m         # Geometric Jacobian + manipulability
│   ├── scara_reach.m            # Minimum / maximum horizontal reach
│   ├── scara_workspace.m        # Workspace sampling and metrics
│   ├── scara_workspace_area.m   # Planar area (polar integral / grid check)
│   ├── scara_plot_workspace.m   # Top-view and 3-D workspace plots
│   └── scara_draw.m             # 3-D rendering of the arm (create / update)
├── examples/
│   ├── demo_forward_kinematics.m
│   ├── demo_workspace_analysis.m
│   └── demo_ik_trajectory.m
├── tests/
│   └── test_scara_kinematics.m  # Unit tests (runtests)
├── docs/
│   └── kinematics.md            # Derivation of all equations
├── .github/workflows/tests.yml  # Runs the tests on every push (GitHub Actions)
├── LICENSE
└── README.md
```

---

## Running the tests

```matlab
>> addpath src
>> results = runtests('tests');
>> table(results)
```

The GitHub Actions workflow in `.github/workflows/tests.yml` runs the same tests on every push and pull request, using the official MathWorks actions.

---

## Requirements

- **MATLAB R2018b or newer** for the GUI (`uifigure`, `uigridlayout`).
- The kinematics library and example scripts also run on older releases (R2017a+).
- **No toolboxes needed.**

Units: the library uses metres and radians. The GUI shows millimetres and degrees.

---

## License

Released under the [MIT License](LICENSE).
