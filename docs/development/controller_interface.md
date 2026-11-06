# Spec: Shared Torque Controllers for Sim and Hardware

Status: draft, implemented step by step on branch `feat/controller-interface`.

## Goal

Run the *same* low-level controller code in MuJoCo and on real hardware, so
that a policy sees the same closed-loop behaviour (compliance, tracking lag,
clipping) in both.

Today the two differ:

| | Joint commands | Cartesian commands |
|---|---|---|
| Sim (`SimRobot`) | MuJoCo `<position>` actuator (PD inside MuJoCo) | Pinocchio IK, then the same position actuator |
| FR3 hardware (`Franka`) | RCS joint PD at 1 kHz (`Franka::joint_controller`) | RCS OSC at 1 kHz (`Franka::osc`) |

## Non-goals

- Hardware is **not** required to use the interface. UR5e, xArm, Flexiv,
  YAM and SO101 keep their vendor servo modes.
- No changes to the Python env / wrapper layer or the action spaces.
  `ControlMode.JOINTS` and `ControlMode.CARTESIAN_*` keep working and are
  routed to whichever controller is installed.
- No whole-body or multi-robot controller. A robot is controlled on its own;
  coordination stays above this layer.
- Controllers written in Python. They are C++ only.

## Concepts

```
policy rate (e.g. 20 Hz)              servo rate (sim: model timestep, FR3: 1 kHz)
  set_goal(goal) ──► Interpolator ──► setpoint ──► Controller ──► joint torque ──► backend
                                         ▲                           (no gravity)
                                ControllerState (q, dq, M, c, g, J, tcp pose, dt)
```

- **ControllerState**: everything a torque controller needs about one robot,
  filled by the *backend* (MuJoCo or libfranka) on every servo tick. All
  Cartesian quantities are in the **robot base frame** at the **TCP**
  (`tcp_offset` applied).
- **Interpolator**: turns sparse goals into a smooth setpoint (position plus
  velocity) on every servo tick.
- **Controller**: owns an interpolator. `set_goal()` is called at policy rate,
  `step(state)` at servo rate, and `step()` returns joint torques.
- **Backend**: builds the `ControllerState`, calls `step()` and applies the
  torque. Only the backend knows the hardware.

### Gravity convention

Controllers return torque **without gravity compensation**, because Franka
compensates gravity in firmware. A backend whose actuators don't do that (the
MuJoCo backend) adds `state.gravity` itself. Coriolis compensation is the
controller's job, using `state.coriolis`.

### Threading

The interfaces aren't thread-safe. A backend that calls `set_goal()` and
`step()` from different threads (FR3) has to serialise those calls itself, as
it already does today with `interpolator_mutex`.

## Interfaces (core package, `include/rcs/`)

`Interpolator.h`

```cpp
struct JointSetpoint     { VectorXd q; VectorXd dq; };
struct CartesianSetpoint { Pose pose; Vector6d twist; };   // twist = [v; w], base frame

class JointInterpolator {
  virtual void reset(const VectorXd& q) = 0;           // hold q, no motion
  virtual void set_goal(const VectorXd& q_goal) = 0;   // start a new segment from the current setpoint
  virtual JointSetpoint step(double dt) = 0;           // advance by dt, return the setpoint
};
class CartesianInterpolator { /* same, with Pose */ };
```

`Controller.h`

```cpp
struct ControllerState {
  VectorXd q, dq;                     // n
  MatrixXd mass;                      // n x n
  VectorXd coriolis;                  // n, C(q, dq) * dq
  VectorXd gravity;                   // n
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;  // TCP, base frame, rows [v; w]
  Pose tcp_pose;                      // base frame
  double dt;                          // seconds since the previous step()
};

class JointController {
  virtual void reset(const ControllerState& state) = 0;   // hold the current configuration
  virtual void set_goal(const VectorXd& q_goal) = 0;
  virtual VectorXd step(const ControllerState& state) = 0; // joint torque without gravity
};
class CartesianController { /* same, with a Pose goal in the base frame */ };
```

Joint and Cartesian are separate interfaces on purpose. Each is about ten
lines, the goal type is checked at compile time, and the backend only has to
know "is the last command a joint or a Cartesian one".

## Where the implementations live

| Implementation | Package | Notes |
|---|---|---|
| `LinearJointInterpolator`, `LinearCartesianInterpolator` | core | Straight line (slerp for rotation) from the current setpoint to the goal over a fixed duration, then hold. Generalises the 7-DoF `LinearPoseTrajInterpolator.h`, which is deleted once FR3 has moved over. |
| `JointPD` | core | `tau = Kp (q_d - q) + Kd (dq_d - dq) + coriolis` |
| `OSC` | core | Decoupled operational-space control with null-space posture: `tau = Jpᵀ Λp (Kp_p e_p + Kd_p (v_d - v)) + Joᵀ Λo (Kp_o e_o + Kd_o (w_d - w)) + N (posture term) + coriolis` |
| `FrankaJointController`, `FrankaOSC` | `rcs_fr3` | The current `Franka::joint_controller` / `Franka::osc` maths moved out of the libfranka lambdas: residual mass, null-space posture, joint-limit avoidance and guard, deadband, `franka::limitRate`, torque clip. The rate limit uses the controller's own previous output instead of `tau_J_d`, so it runs in sim too. |
| Ruckig interpolator | open | See open questions. |

## Backends

### MuJoCo (`SimRobot`)

- New: `set_joint_controller(shared_ptr<JointController>)` and
  `set_cartesian_controller(shared_ptr<CartesianController>)`. Without a
  controller, `SimRobot` behaves exactly as today (native MuJoCo PD).
- With a controller installed:
  - On install, the arm actuators are switched to torque mode at runtime, the
    same way `set_config` already rewrites `gainprm`/`biasprm` (gain 1,
    bias 0). The torque is clamped to the actuator's `forcerange`/`ctrlrange`.
  - The controller is stepped from a per-step `Sim` callback, i.e. after
    `mj_step1` (where `qM`, `qfrc_bias` and kinematics are current) and before
    `mj_step2`. That is the same position as robosuite's
    `step1 → _pre_action → step2`.
  - State: `q`/`dq` from the robot's joints; `mass` from `mj_fullM`,
    restricted to the robot's DoFs; `gravity` from `qfrc_gravcomp`-style
    evaluation (RNE with zero velocity); `coriolis = qfrc_bias - gravity`;
    `jacobian` from `mj_jac` at the TCP point, rotated into the base frame.
  - `ctrl = step(state) + gravity`.
  - `set_joint_position(q)` goes to the joint controller if one is installed.
    `set_cartesian_position(pose)` goes to the Cartesian controller if one is
    installed, otherwise IK and the joint path as today. Switching between
    the two calls `reset()` on the newly active controller.
  - The convergence callback (`step_until_convergence`) compares the TCP pose
    instead of joint angles while a Cartesian controller is active.

### FR3 hardware (`Franka`)

- The bodies of the two `robot.control(...)` lambdas become: fill the
  `ControllerState` from `franka::Model` and `RobotState`, call
  `controller->step()`, return the torque.
- By default `Franka` builds a `FrankaJointController` / `FrankaOSC` from
  `FrankaConfig`, so the Python API and configs are unchanged.
- The missing Coriolis compensation in the current OSC and joint controller is
  fixed in a separate commit, because it changes how the real robot behaves.

### Using hardware controllers in sim

`FrankaOSC` only needs a `ControllerState`. With `rcs_fr3` installed it can
be passed to `SimRobot.set_cartesian_controller()`, so the sim runs the exact
FR3 controller, including its clipping. Franka firmware behaviour (reflexes,
collision thresholds, internal torque tracking) is **not** modelled.

## Implementation steps (each step is reviewable on its own)

1. Spec and interfaces (header only, no behaviour change).
2. Linear interpolators in core, plus pybind and unit tests.
3. `JointPD` and `OSC` in core, plus pybind.
4. MuJoCo backend in `SimRobot`. Sim tests: hold under gravity, reach a joint
   goal, reach a Cartesian goal, switch modes.
5. `FrankaJointController` / `FrankaOSC` in `rcs_fr3`; `Franka` uses them;
   delete `LinearPoseTrajInterpolator.h`. Sim test with the FR3 controllers.
6. Fix: add Coriolis compensation to the FR3 controllers (needs testing on
   hardware).

## Open questions

- **Ruckig** (MIT community version) as a jerk-limited interpolator: core or
  an optional extension? It is only an extra `JointInterpolator`, so it can be
  added later without interface changes.
- **Timestep**: FR3 runs at 1 kHz, the sim model at 500 Hz (default 2 ms).
  Use 1 ms in torque-controlled scenes so gains carry over?
- **Linear interpolator start point**: the current FR3 interpolator starts
  each segment at the *previous goal*. The new one starts at the *current
  setpoint*, which avoids a setpoint jump when a goal arrives before the last
  segment is finished. That is a small behaviour change for FR3.
- **FR3 default `torque_limit` of 5 Nm** is very conservative and will
  saturate in sim with tuned gains.
