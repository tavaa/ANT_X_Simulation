# ANT-X Simulation — MATLAB

> LTV-MPC trajectory tracking for the ANT-X nano quadrotor (270 g) via differential flatness.  
> Euler 12-state and quaternion 13-state models · Circle, Lemniscate, Spiral trajectories · Full closed-loop results.

![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%2B-orange?logo=mathworks&logoColor=white)
![Optimization Toolbox](https://img.shields.io/badge/Optimization%20Toolbox-required-blue?logo=mathworks&logoColor=white)
![ODE45](https://img.shields.io/badge/ode45-numerical%20integration-lightgrey?logo=mathworks&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)

---

> **🚀 Version 2 — Major Updates**
>
> V2 contains updated version of the original MATLAB simulation pipeline. Compared with **v1**, the main changes are:
>
> * **Trajectory recalibration** — Updated trajectory parameters for more regular profiles and better compatibility with the laboratory flight volume.
> * **Improved PWC reference generation** — Replaced pointwise input sampling with the **exact integral average of the continuous input** over each sampling interval, reducing bias in higher-order derivatives.
> * **16-state nonlinear model** — Introduced a new model including **first-order motor dynamics and individual rotor thrust states**.
> * **Motor allocation and mixer** — Added explicit **allocation and mixer matrices**, including the conversion between virtual thrust/moments and individual rotor thrusts.
> * **Parameter identification framework** — Added a preliminary parameter estimation procedure and a detailed experimental identification procedure intended for laboratory deployment.
> * **Updated MPC formulation** — Replaced `quadprog` with **OSQP**, introduced direct physical constraints on individual rotor thrusts, systematically defined the weighting matrices, and adopted **exact ZOH discretization**.
> * **More realistic closed-loop simulation** — Added **actuation/computation latency** and **3-second hover tails** for Perturbation and Origin initial conditions, making the simulations more representative of the planned flight tests.
> * **Extended validation** — Added quantitative tracking, computational-performance, and motor-allocation checks for the updated B01 formulation.



## Introduction

This repository contains the first MATLAB simulation pipeline for the [ANT-X](https://ant-x.gitlab.io) UAV.

The pipeline covers the full stack from nonlinear modelling to closed-loop control:

1. Nonlinear rigid-body dynamic model (NED/FRD, Euler Z-Y-X or unit quaternion)
2. Differential flatness map — state and input reconstruction from flat outputs σ = [x, y, z, ψ]
3. Feasible trajectory generation (Circle, Lemniscate, Spiral) with offline PWC reference
4. Open-loop validation via `ode45`
5. LTV-MPC closed-loop control via sequential linearisation + `quadprog`
6. Per-run CSV export and quantitative tracking metrics

Detailed documentation is in [`documentation/`](documentation/).

---

## Prerequisites

| Requirement | Notes |
|---|---|
| MATLAB R2024b or later | Earlier versions may work |
| Optimization Toolbox | Required for `quadprog` |

---

## How to Use

### 1. Clone the repository

```bash
git clone https://github.com/tavaa/ANT_X_Simulation.git
cd ANT_X_Simulation
```

### 2. Open MATLAB and add the project to the path

```matlab
addpath(genpath('.'))
```

### 3. Run open-loop tests (DoF validation)

```matlab
% Individual degree-of-freedom tests
run('scripts/openloop/run_pitch_test.m')
run('scripts/openloop/run_roll_test.m')
run('scripts/openloop/run_thrust_test.m')
run('scripts/openloop/run_yaw_test.m')
```

### 4. Run open-loop trajectory tests

```matlab
% Circle, Lemniscate, Spiral — continuous vs PWC comparison
run('scripts/openloop/run_circle_openloop.m')
run('scripts/openloop/run_lemniscate_openloop.m')
run('scripts/openloop/run_spiral_openloop.m')
```

### 5. Run closed-loop MPC simulations

```matlab
% Full closed-loop: all trajectories × all initial conditions
run('scripts/MPCSimulationLoop.m')
```

Results are saved to `results/<trajectory>/<initial_condition>/`.

---

## Model

The quadrotor is modelled as a rigid body under Newton-Euler equations in NED/FRD frames.

**12-state model** (`A00_SimplifiedModel.m`) — Euler Z-Y-X attitude:

```
x = [xI  yI  zI  ẋB  ẏB  żB  φ  θ  ψ  p  q  r]ᵀ ∈ ℝ¹²
u = [T  L  M  N]ᵀ ∈ ℝ⁴
```

**13-state model** (`A01_SimplifiedModel_quat.m`) — unit quaternion attitude (singularity-free).

**Parameters** — m = 0.270 kg, b = 0.08 m, J = diag(0.00307, 0.00307, 0.00239) kg·m².  
Inertia values from Cavagnini (2021); SysID aerodynamic parameters from El Omari (2023).

> **NED sign convention:** z-down. Climb → negative zNED. Forward acceleration → θ < 0.

---

## Flatness Map

Flat outputs: **σ = [xI, yI, zI, ψ]ᵀ**

State and inputs are reconstructed algebraically from σ and its time derivatives up to snap (4th order), without integrating the rotational dynamics:

| Quantity | Requires |
|---|---|
| Position, velocity | (position, velocities) |
| Attitude (R, Euler/quat) | (accellerations) |
| Body rates ωB |  (jerk) |
| T, L, M, N |  (snap) |

---

## Trajectories

| Trajectory | Shape | Parameters |
|---|---|---|
| **Circle** | Uniform circular, constant altitude | R = 1.0 m, ω = 1.0 rad/s, h = −1.0 m |
| **Lemniscate** (Gerono) | Figure-8 planar | R = 1.0 m, ω = 0.5 rad/s, h = −1.0 m |
| **Spiral** | Elliptic helix, descending | Rx = 2.0 m, Ry = 1.0 m, ω = 0.5 rad/s, vz = 0.2 m/s |

Each trajectory provides position, velocity, acceleration, jerk, and snap in closed form. Offline PWC references are generated via `GeneratePWC_reference.m` using `ode45`.

Three initial conditions are tested per trajectory:

| Condition | Description |
|---|---|
| `OnReference` | Drone initialised exactly at xref(0) |
| `Perturbation` | +0.20 m offset on x, −0.20 m on y; velocities/rates zeroed |
| `Origin` | Drone at ground (zI = −0.02 m), all states zero |

---

## MPC

**Method:** LTV-MPC with sequential linearisation along the reference trajectory (Kunz, Huck & Summers, ECC 2013).

| Parameter | Value |
|---|---|
| Sampling time Ts | 0.05 s |
| Prediction horizon p | 18 steps |
| State cost Q = Qf | diag(50,50,50, 8,8,12, 8,8,10, 1,1,1) |
| Input cost R | diag(2,2,2,2) |
| QP solver | `quadprog` (MATLAB Optimization Toolbox) |

At each step: linearise → Euler-discretise → build sparse QP → solve → apply optimal u → warm-start next iteration.

**State constraints** (from El Omari Table 5.3):

```
-3 ≤ ẋB, ẏB ≤ 3 m/s      -3 ≤ żB ≤ 1 m/s
|φ|, |θ| ≤ 35°             |p|, |q|, |r| ≤ 600°/s
0 ≤ T ≤ 2mg ≈ 5.30 N      |L|, |M| ≤ 0.15 Nm      |N| ≤ 0.05 Nm
```

**Steady-state tracking results** (Ts = 0.05 s):

| Scenario | Condition | RMSEpos SS [m] | tconv [s] | QP mean [ms] |
|---|---|---|---|---|
| Circle | Perturbation | 0.0091 | 1.50 | 9.04 |
| Lemniscate | Perturbation | 0.0064 | 1.35 | 8.55 |
| Spiral | Origin | 0.0049 | 1.85 | 8.88 |

> `quadprog` is not real-time embeddable.

---

## Repository Structure

```
antx-simulation/
├── models/               # non-linear models
├── flatness/             # flatness input/state reconstruction
├── trajectories/         # Shape classes (Circle, Lemniscate, Spiral, DoF tests)
├── control/
│   ├── MPC/              # QPBuilder, LTV-MPC controller
├── scripts/              # Runnable simulation scripts
├── results/              # Auto-generated output (CSV, plots, GIFs)
├── documentation/        # Full technical report (PDF)
└── README.md
```

---

## Known Limitations

- Rigid body only — no rotor drag, blade flapping, or motor/ESC dynamics
- Instantaneous actuation assumed
- Motor mixing matrix not implemented (CT, CQ unknown)
- Inertia values estimated from a similar platform; experimental re-identification needed
- No EKF — full noiseless state assumed
- `Qf = Q` is a pragmatic choice inherited from Kunz et al.; asymptotic stability not guaranteed

---

## References

1. ANT-X Development Team, *ANT-X drone platform — software tools documentation*, 2024.
2. G. Cavagnini, *Identification and control of a nano quadrotor*, MSc thesis, Politecnico di Milano, 2021.
3. L. Kunz, M. Huck, T. H. Summers, "Fast model predictive control of miniature helicopters," *ECC 2013*, pp. 1377–1382.
4. S. El Omari, *Multirotor UAV control via dynamic inversion*, MSc thesis, Politecnico di Milano, 2023.
5. D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors," *ICRA 2011*, pp. 2520–2525.
6. M. Faessler, A. Franchi, D. Scaramuzza, "Differential flatness of quadrotor dynamics subject to rotor drag," *RA-L*, vol. 3, pp. 620–626, 2018.


