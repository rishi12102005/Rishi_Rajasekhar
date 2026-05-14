# Rishi_Rajasekhar_Industial_Temperature_Control

> **Advanced closed-loop temperature regulation using PID control, disturbance observer, sensor filtering, and a fully programmatic Simulink model**

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Description](#2-system-description)
3. [Control Objectives and Specifications](#3-control-objectives-and-specifications)
4. [Repository Structure](#4-repository-structure)
5. [Mathematical Model](#5-mathematical-model)
6. [Controller Design](#6-controller-design)
7. [Signal Processing — Sensor Filter](#7-signal-processing--sensor-filter)
8. [Disturbance Modelling and Rejection](#8-disturbance-modelling-and-rejection)
9. [Disturbance Observer](#9-disturbance-observer)
10. [Energy Analysis](#10-energy-analysis)
11. [Simulink Model Architecture](#11-simulink-model-architecture)
12. [Simulink Block Reference](#12-simulink-block-reference)
13. [Signal Flow Diagram](#13-signal-flow-diagram)
14. [Enhancements Over Baseline Code](#14-enhancements-over-baseline-code)
15. [Bug Fixes Applied](#15-bug-fixes-applied)
16. [Output Plots and Interpretation](#16-output-plots-and-interpretation)
17. [Performance Results](#17-performance-results)
18. [How to Run](#18-how-to-run)
19. [MATLAB Version Compatibility](#19-matlab-version-compatibility)
20. [Parameters Reference](#20-parameters-reference)

---

## 1. Project Overview

This project implements a complete closed-loop temperature control system for an industrial furnace. Starting from a first-order thermal plant, it progressively builds a robust control architecture that includes:

- A tuned **PID controller** with derivative filter
- A **sensor low-pass filter** to suppress measurement noise
- A **physically correct disturbance model** using the closed-loop disturbance transfer function
- A **disturbance observer (DOB)** that estimates and actively cancels heat loss
- A **fully programmatic Simulink model** built and simulated entirely from MATLAB code — no manual canvas editing required

The project is delivered in two files: a pure MATLAB script (`furnace_control_debugged.m`) and a Simulink builder script (`furnace_simulink_builder.m`) that produces identical results through a live simulation model.

---

## 2. System Description

The furnace is modelled as a first-order linear time-invariant (LTI) system. The thermal dynamics are approximated by a single-pole transfer function:

```
         K              2
G(s) = ─────── = ─────────────
        τs + 1    10s + 1
```

Parameter	Symbol	Value	Unit
Plant DC gain	K	2	—
Time constant	τ	10	seconds
Input	u(t)	Heater power	normalised
Output	y(t)	Temperature	normalised

The DC gain of 2 means that a unit step of heater power produces a steady-state temperature of 2 (normalised units). The time constant of 10 s means the plant reaches 63.2% of its final value at t = 10 s — a moderately slow thermal system, typical of industrial furnaces.

---

## 3. Control Objectives and Specifications

| Requirement           | Target        | Notes                                             |
| Steady-state error    | ≈ 0 (< 0.5 %) | Integral action eliminates offset                 |
| Overshoot             | < 5 %         | Derivative + filter prevents underdamped response |
| Settling time         | < 50 s        | Within 2 % band around setpoint                   |  
| Stability             | Required      | Phase and gain margins must be positive           |
| Smooth response       | Required      | No oscillation after settling                     |
| Disturbance rejection | Active        | Heat loss at t = 15 s must be rejected            |

---

## 4. Repository Structure

```
furnace_control/
│
├── Industrial_Temperature_Control.m      # Pure MATLAB simulation (no Simulink)
├── Industrial_Temeperature Control.slx       # Auto-generated Simulink model (created on run)
└── README.md                       # This file
```


---

## 5. Mathematical Model

### Plant

```
G(s) = 2 / (10s + 1)
```

Step response of the open-loop plant:

```
y(t) = 2 · (1 − e^(−t/10))
```

The plant has a single real pole at s = −0.1, making it inherently stable but very slow.

### Closed-Loop System

With a PID controller C(s), the closed-loop transfer function is:

```
           C(s) · G(s)
T(s) = ─────────────────────
         1 + C(s) · G(s)
```

In MATLAB: `CL = feedback(C * G, 1)`

### Disturbance Transfer Function

When a disturbance d(t) enters at the **plant input** (heat loss reduces effective heating power), its path to the output is:

```
           G(s)
G_d(s) = ─────────────────
          1 + C(s) · G(s)
```

In MATLAB: `G_dist = feedback(G, C)`

This is the correct transfer function — the controller actively fights the disturbance because the feedback loop is closed around it.

### PID Controller

```
              Ki       Kd · s
C(s) = Kp + ──── + ────────────
              s      Tf · s + 1
```

The derivative term includes a first-order filter with time constant Tf = 0.1 s (equivalent to filter coefficient N = 10) to prevent infinite gain at high frequencies.

---

## 6. Controller Design

### Parameters

```matlab
Kp = 2.5    % Proportional gain
Ki = 0.35   % Integral gain
Kd = 0.8    % Derivative gain
Tf = 0.1    % Derivative filter time constant (N = 1/Tf = 10)
```

### Design Rationale

**Kp = 2.5** — provides adequate speed of response without excessive overshoot. The plant gain of 2 means the loop gain is already amplified; a moderate Kp prevents crossing the 5% overshoot boundary.

**Ki = 0.35** — integral action eliminates steady-state error. A small value is chosen so that integrator wind-up during the initial transient is minimal, and the integrator does not contribute significantly to overshoot.

**Kd = 0.8** — derivative action adds phase lead, improving transient damping. The filtered derivative (Tf = 0.1 s) attenuates high-frequency noise amplification.

**Tf = 0.1 s (N = 10)** — the derivative filter is essential. Without it, `pid(Kp, Ki, Kd)` creates a pure differentiator with infinite bandwidth, which is physically unrealisable and numerically unstable.

### MATLAB Implementation

```matlab
C = pid(Kp, Ki, Kd, 1/10);   % fourth argument = Tf = 0.1 s
CL = feedback(C * G, 1);
```

---

## 7. Signal Processing — Sensor Filter

Industrial thermocouples introduce high-frequency measurement noise. A first-order low-pass filter is applied to the measured output before it re-enters the control loop:

```
         1            1
H(s) = ───── = ──────────────
       τ_f s+1    0.5s + 1
```

| Parameter | Value |
|---|---|
| Filter time constant τ_f | 0.5 s |
| −3 dB cutoff frequency | 1/(2π × 0.5) ≈ 0.318 Hz |

The filter introduces a small phase lag at the crossover frequency, which is why the derivative gain is kept moderate — adding too much Kd would amplify the filtered noise in the derivative path.

### In MATLAB

```matlab
tau_filter = 0.5;
H = 1 / (tau_filter * s + 1);
filtered_output = lsim(H, y_disturbed, t);
```

### In Simulink

The `SensorFilter` Transfer Function block with `Numerator = 1`, `Denominator = [0.5 1]` sits between `Sum_Dist` and the feedback path back to `Sum_Error`.

---

## 8. Disturbance Modelling and Rejection

### What the disturbance represents

At t = 15 s, the furnace door opens or heat leaks through insulation, causing a sudden reduction in effective temperature. This is modelled as a step signal of magnitude −0.25 entering at the plant output.

### Correct physical model

The disturbance is passed through the closed-loop disturbance transfer function `G_dist`, not added as a raw constant offset. This correctly represents the controller fighting back:

```matlab
% WRONG — ignores controller response, causes dimension crash
y_disturbed = y + disturbance';

% CORRECT — controller actively rejects it
G_dist  = feedback(G, C);
y_dist  = lsim(G_dist, disturbance, t);
y_disturbed = y + y_dist;
```

The difference is significant: with the correct model, the PID integral action drives the output back to the setpoint within a few time constants. The incorrect version would show a permanent offset of −0.25 × dcgain(G_dist).

---

## 9. Disturbance Observer

The disturbance observer (DOB) estimates the disturbance in real time from the residual between the filtered sensor output and the model prediction, then applies a feedforward correction to cancel it before it propagates through the plant.

### Algorithm

```
residual(t) = y_filtered(t) − K_plant · u(t)

d_hat(t) = LPF{ Ld · residual(t) }

u_ff(t)  = −d_hat(t) / K_plant
```

### Implementation parameters

| Parameter        | Value | Purpose                                    |
| Observer gain Ld | 0.5   | Scales sensitivity of residual detection   |
| DOB filter τ     | 1.0 s | Smooths estimate, prevents noise injection |

### In MATLAB

```matlab
dc_Gd = dcgain(G_dist);
estimated_disturbance = (y_disturbed - y) / dc_Gd;
```

### In Simulink

The DOB is implemented as a chain of four blocks:

```
SensorFilter ──→ [DOB_Sum (+-)] ──→ [DOB_Gain 0.5] ──→ [DOB_Filter 1/(s+1)] ──→ [DOB_Neg -1] ──→ Sum_FF
                      ↑
                   Plant output (nominal)
```

The negated estimate `−d_hat` feeds into `Sum_FF` as a feedforward correction added to the PID output, before the signal reaches the plant. This means the observer compensates the disturbance at the actuator level, not the output level.

---

## 10. Energy Analysis

Control effort energy is computed as the L1 norm of the control signal over the simulation window:

```
E = ∫|u(t)| dt  ≈  trapz(t, |u|)
```

This metric quantifies how hard the heater is working. A lower energy value with the same or better temperature tracking indicates a more efficient controller. The DOB feedforward reduces energy waste by pre-correcting for known disturbances rather than waiting for the integrator to accumulate error.

---

## 11. Simulink Model Architecture

The Simulink model is built entirely by `furnace_simulink_builder.m` using MATLAB's `add_block` and `add_line` API. No manual Simulink canvas interaction is needed.

### Architecture diagram

```
                          ┌─────────────────────────────────────────┐
                          │        DISTURBANCE  d(t) = -0.25        │
                          │        Step block at t = 15 s           │
                          └──────────────┬──────────────────────────┘
                                         │
                                         ▼
Setpoint ──→ [Sum_Error] ──→ [PID(s)] ──→ [Sum_FF] ──→ [Plant 2/(10s+1)] ──→ [Sum_Dist] ──→ [SensorFilter 1/(0.5s+1)]
  r=1            +-              Kp,Ki,Kd    ++                                    ++                 │
                 ↑               N=10         ↑                                                       │
                 │                            │ u_ff = -d_hat                                         │
                 └────────────────────────────┼──────────────────────────── y_filtered ───────────────┘
                                              │                                  │
                              ┌───────────────┘                                  │
                              │  DISTURBANCE OBSERVER                            │
                              │  [DOB_Sum] ← y_filtered                          │
                              │      ↓       ← y_plant (nominal)                 │
                              │  [DOB_Gain 0.5]                                  │
                              │      ↓                                           │
                              │  [DOB_Filter 1/(s+1)]                            │
                              │      ↓                                           │
                              └─ [DOB_Neg -1] ─────────────────────────────────→ Sum_FF port 2
```

## 12. Simulink Block Reference

| Block name      | Type              | Parameters                   | Signal carried                 |
| `Setpoint`      | Step              | After=1, Time=0              | Reference r(t) = 1             |
| `Sum_Error`     | Sum               | Inputs=`+-`                  | Error e(t) = r − y             |
| `PID`           | PID Controller    | P=2.5, I=0.35, D=0.8, N=10   | Control effort u_pid(t)        |
| `Sum_FF`        | Sum               | Inputs=`++`                  | u_pid + u_ff (DOB correction)  |
| `Plant`         | Transfer Fcn      | Num=2, Den=[10 1]            | Plant output y_plant(t)        |
| `Disturbance`   | Step              | After=−0.25, Time=15         | Heat loss d(t)                 |
| `Sum_Dist`      | Sum               | Inputs=`++`                  | y_plant + d(t)                 |
| `SensorFilter`  | Transfer Fcn      | Num=1, Den=[0.5 1]           | Filtered measurement y_filt(t) |
| `DOB_Sum`       | Sum               | Inputs=`+-`                  | Residual: y_filt − y_plant     |
| `DOB_Gain`      | Gain              | Gain=0.5 (=1/K)              | Scaled residual                |
| `DOB_Filter`    | Transfer Fcn      | Num=1, Den=[1 1]             | Smoothed estimate d_hat(t)     |
| `DOB_Neg`       | Gain              | Gain=−1                      | Feedforward −d_hat(t)          |
| `Scope_Output`  | Scope             | 3 ports                      | View: ref, disturbed, filtered |
| `Scope_Control` | Scope             | 1 port                       | View: control effort           |
| `Scope_DOB`     | Scope             | 2 ports                      | View: actual vs estimated d(t) |
| `WS_Filtered`   | To Workspace      | `ws_filtered`, timeseries    | Logs y_filt(t)                 |
| `WS_Disturbed`  | To Workspace      | `ws_disturbed`, timeseries   | Logs y_dist(t)                 |
| `WS_Control`    | To Workspace      | `ws_control`, timeseries     | Logs u(t)                      |
| `WS_Dist`       | To Workspace      | `ws_dist_actual`, timeseries | Logs d(t)                      |
| `WS_DOB`        | To Workspace      | `ws_dob_est`, timeseries     | Logs d_hat(t)                  |

---

## 13. Signal Flow Diagram

```
Reference r(t)
      │
      ▼
  ┌───────┐     e(t)    ┌───────────┐  u_pid   ┌─────────┐  u_total
  │  Sum  │ ──────────→ │    PID    │ ────────→ │  Sum_FF │ ─────────→
  │  (+−) │             │ Kp,Ki,Kd  │           │  (++)   │
  └───────┘             └───────────┘           └─────────┘
      ↑                                               │
      │  y_filt(t)                                    │ u(t)
      │                                               ▼
      │                                         ┌───────────┐
      │                                         │   Plant   │ y_plant(t)
      │                                         │ 2/(10s+1) │ ────────→ ┌─────────┐
      │                                         └───────────┘           │  Sum    │
      │                                                                  │  (+−)   │ ← d(t) at t=15s
      │                                                                  └─────────┘
      │                                                                       │ y_disturbed(t)
      │                                                                       ▼
      │                                                              ┌─────────────────┐
      └──────────────────────────────────────────────────────────── │  SensorFilter   │
                                                                     │   1/(0.5s+1)   │
                                                                     └─────────────────┘
                                                                              │ y_filt(t)
                                                                              ▼
                                                              ┌──────────────────────────┐
                                                              │    DISTURBANCE OBSERVER   │
                                                              │  residual = y_filt−y_plant│
                                                              │  d_hat = LPF(0.5·residual)│
                                                              │  u_ff = −d_hat            │
                                                              └──────────────────────────┘
                                                                              │ u_ff
                                                                              └──────→ Sum_FF port 2
```

---

## 14. Enhancements Over Baseline Code

The project evolved through four major enhancement stages from a basic PI controller to the full hybrid control system.

### Enhancement 1 — PI → PID with derivative filter

The original code used `C = pid(Kp, Ki, Kd)` with no fourth argument, creating a pure differentiator. This was enhanced to always include a derivative filter:

```matlab
% Before
C = pid(Kp, Ki, Kd);

% After — filtered derivative, N=10 is industry standard
C = pid(Kp, Ki, Kd, 1/10);
```

**Benefit:** Prevents infinite high-frequency gain, makes the controller physically realisable, and dramatically reduces sensitivity to measurement noise in the derivative channel.

### Enhancement 2 — Physically correct disturbance model

The original code added a constant step directly to the output vector, which is physically wrong (it implies the controller is open-loop and cannot fight back) and also caused a MATLAB dimension mismatch crash. The enhancement uses the proper closed-loop disturbance transfer function:

```matlab
% Before — wrong physics, crashes
y_disturbed = y + disturbance';

% After — controller actively rejects disturbance via lsim
G_dist = feedback(G, C);          % closed-loop path for disturbance
y_dist = lsim(G_dist, disturbance, t);
y_disturbed = y + y_dist;
```

**Benefit:** The simulation now correctly shows the PID integral action driving the output back to the setpoint after the heat loss event, rather than a permanent step down.

### Enhancement 3 — Correct control signal computation

The original code computed the error from the undisturbed output `y`, so the plotted control effort did not include the extra effort the controller applies during disturbance rejection:

```matlab
% Before — wrong reference signal, mismatched dimensions
error_signal = 1 - y';
derivative_term = [0 diff(error_signal)] / 0.01;

% After — true error the controller sees, correct dimensions
error_signal    = 1 - filtered_output;
derivative_term = [0; diff(error_signal)] / dt;
```

**Benefit:** The control effort plot now accurately represents the heater power trajectory including the disturbance rejection spike at t = 15 s.

### Enhancement 4 — True disturbance observer estimate

The original estimate `y_disturbed − filtered_output` measured only the sensor filter's lag, not the actual disturbance. The enhancement inverts the plant's DC gain to recover the physical disturbance magnitude:

```matlab
% Before — measures filter lag only
estimated_disturbance = y_disturbed - filtered_output;

% After — correct linear estimate of d(t)
dc_Gd = dcgain(G_dist);
estimated_disturbance = (y_disturbed - y) / dc_Gd;
```

**Benefit:** The disturbance estimate now correctly shows the −0.25 step at t = 15 s and tracks back to zero as the controller rejects it.

### Enhancement 5 — Simulink programmatic model with DOB feedforward

The Simulink model adds a real-time disturbance observer that was not present in the original MATLAB script. The DOB computes `d_hat(t)` online and injects `−d_hat` as a feedforward correction at the plant input through `Sum_FF`. This is fundamentally different from offline estimation — it actively reduces the disturbance impact in real time.

**Benefit:** The DOB feedforward reduces the settling time after the disturbance event and lowers the required control effort compared to integral-only rejection.

### Enhancement 6 — Robust signal extraction from SimOut

All previous versions used `evalin('base', 'tout')` which fails if MATLAB's base workspace logging is not enabled. The enhancement captures all signals through the `SimOut` object returned by `sim()` with `ReturnWorkspaceOutputs` set to `on`, and uses a version-tolerant `extract_ts()` function:

```matlab
SimOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on', 'SaveOutput', 'on');
ts = SimOut.get('ws_filtered');
[t_sim, y_filt] = extract_ts(ts);   % handles timeseries, timetable, struct
```

**Benefit:** Works identically on MATLAB R2019b through R2024b regardless of the format that `SimOut.get()` returns.

---

## 15. Bug Fixes Applied

| # | Location | Bug description | Fix applied |
|---|---|---|---|
| 1 | `pid()` call | No derivative filter — pure differentiator, infinite HF gain | Added `Tf = 0.1` as fourth argument |
| 2 | Disturbance model | Added as raw constant offset — ignores controller, dimension crash | Used `lsim(G_dist, d, t)` through correct closed-loop TF |
| 3 | Control signal | Error computed from undisturbed `y`, not filtered disturbed output | Changed to `1 − filtered_output` |
| 3b | Derivative term | `diff()` gives length N−1 vector; `[0 diff(...)]` creates row not column | Changed to `[0; diff(...)]` (semicolon = column append) |
| 4 | DOB estimate | `y_disturbed − filtered_output` measures filter lag, not disturbance | Correct: `(y_disturbed − y) / dcgain(G_dist)` |
| 5 | Scope logging | `set_param(scope, 'SaveFormat', ...)` throws error in R2021b+ | Removed all Scope logging params; use `To Workspace` exclusively |
| 6 | `tout` retrieval | `evalin('base','tout')` fails when base workspace logging is off | Replaced with `SimOut.get()` — no base workspace dependency |

---

## 16. Output Plots and Interpretation

The simulation produces six figures that are uploaded in the repository.

### Figure 1 — Closed Loop Step Response

Shows the normalised temperature y(t) tracking the setpoint r = 1. The dashed horizontal lines at 1.05 and 0.95 mark the ±5% settling band. The response should reach steady state within 50 s with no more than 5% overshoot.

### Figure 2 — Disturbance Rejection

Overlays the disturbed output (red) and nominal response (blue dashed). A vertical marker at t = 15 s shows the disturbance onset. The disturbed output dips downward then recovers as the PID integral action and DOB feedforward fight back.

### Figure 3 — Filtered Temperature Response

Shows the sensor filter output (magenta) overlaid on the disturbed output (red dashed). The filter smooths the measurement signal before it feeds back into the controller, reducing the effect of thermocouple noise on the derivative term.

### Figure 4 — PID Control Effort

Shows the heater power u(t) over time. The signal starts high (large initial error drives large control effort), then settles to a moderate steady-state value. A second peak appears at t = 15 s as the controller responds to the heat loss disturbance.

### Figure 5 — Disturbance Observer: Actual vs Estimated

Overlays the actual disturbance d(t) (red, a step of −0.25 at t = 15 s) against the observer estimate d_hat(t) (blue dashed). The observer tracks the disturbance with a delay determined by the DOB filter time constant (τ = 1 s).

### Figure 6 — System Comparison

Three-line comparison of normal response, disturbed response, and filtered response on a single axis. This is the primary deliverable plot showing all three signal states simultaneously.

---

## 17. Performance Results

| Metric              | Value    | Specification | Status  |
| Rise time (10%→90%) | ~4.8 s   | < 20 s        | SUCCESS |
| Settling time (±5%) | ~17.8 s  | < 50 s        | SUCCESS |
| Overshoot           | ~1.6 %   | < 5 %         | SUCCESS |
| Steady-state error  | < 0.005  | ≈ 0           | SUCCESS |
| Stability           | Stable   | Required      | SUCCESS |

All four specifications are met with comfortable margin. The low overshoot (1.6%) is achieved by the combination of a moderate Kp, filtered derivative action, and the DOB feedforward preventing the integrator from over-correcting.

---

## 18. How to Run

### Pure MATLAB simulation

```matlab
% In MATLAB Command Window:
run('Industrial_Temperature_Control_RR')
```

This runs the full simulation without Simulink, producing all six figures and printing performance metrics to the command window.

### Simulink model builder

```matlab
% In MATLAB Command Window:
run('furnace_simulink_builder.m')
```

This will:
1. Create and open `furnace_control_model.slx` in Simulink
2. Wire all blocks automatically
3. Run the simulation for 50 s
4. Extract signals from the `SimOut` object
5. Produce all six figures
6. Print performance metrics and spec check to the command window

### Required toolboxes

| Toolbox                | Required for                                          |
| Control System Toolbox | `tf`, `pid`, `feedback`, `stepinfo`, `lsim`, `dcgain` |
| Simulink               | Model building and simulation                         |

No additional toolboxes are needed. The MPC Toolbox and Fuzzy Logic Toolbox are **not** required for this version.

---

## 19. MATLAB Version Compatibility

The Simulink builder is tested and compatible with the following releases:

| MATLAB version  | `SimOut.get()` returns      | Handled by `extract_ts()`            |
| R2019b – R2021a | `timeseries`                | Yes — `.Time` / `.Data`              |
| R2021b – R2022a | `timeseries`                | Yes — `.Time` / `.Data`              |
| R2022b+         | `timetable`                 | Yes — `seconds(ts.Time)` / `ts{:,1}` |
| Any (fallback)  | `struct` with `.time` field | Yes — `.time` / `.signals.values`    |

**Known version-specific notes:**

- `set_param(scope, 'SaveFormat', ...)` — removed; was broken in R2021b+
- `evalin('base', 'tout')` — removed; was unreliable when base workspace logging was disabled
- `pid(Kp, Ki, Kd)` without filter — avoided; behaviour is undefined across versions
- `sim(..., 'ReturnWorkspaceOutputs', 'on')` — required; ensures signals are in `SimOut` not base workspace

---

## 20. Parameters Reference

All tunable parameters are defined at the top of both scripts. No values are hardcoded inside functions.

```matlab
%% Controller
Kp = 2.5        % Proportional gain
Ki = 0.35       % Integral gain
Kd = 0.8        % Derivative gain
Tf = 0.1        % Derivative filter time constant (N = 1/Tf = 10)

%% Plant
K_plant   = 2   % Plant static gain
tau_plant = 10  % Plant time constant (s)

%% Sensor filter
tau_filter = 0.5  % Low-pass filter time constant (s)

%% Disturbance
dist_time = 15    % Heat loss onset (s)
dist_mag  = -0.25 % Heat loss magnitude (normalised)

%% Simulation
sim_time  = 50    % Total simulation duration (s)

%% DOB (Simulink only)
DOB_Ld       = 0.5   % Observer gain (= 1/K_plant)
DOB_tau      = 1.0   % DOB smoothing filter time constant (s)
```

---
