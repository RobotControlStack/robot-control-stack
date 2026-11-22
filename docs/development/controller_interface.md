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
                                         ▲
                     backend (q, dq) ──► PinocchioDynamics
```

- **ControllerState**: everything a torque controller needs about one robot,
  computed on every servo tick by `PinocchioDynamics` from the `q` and `dq`
  that the backend measured. All Cartesian quantities are in the **robot base
  frame** at the **TCP** (`tcp_offset` applied).
- **PinocchioDynamics**: the robot's dynamics model. Sim and real use the
  same class, so both run the same dynamics code and not only the same
  controller.
- **Interpolator**: turns sparse goals into a smooth setpoint (position plus
  velocity) on every servo tick.
- **Controller**: owns an interpolator. `set_goal()` is called at policy rate,
  `step(state)` at servo rate, and `step()` returns joint torques.
- **Backend**: reads `q` and `dq`, calls `PinocchioDynamics` and `step()`,
  and applies the torque. Only the backend knows the hardware.

### Gravity convention

Controllers return torque **without gravity compensation**. Gravity is
compensated the way the real robot does it:

- **Real robot**: the firmware compensates the arm plus the tool configured on
  the robot (for Franka: the end effector set in Desk). The backend adds
  nothing.
- **Sim**: the backend plays the firmware's part and adds `state.gravity`,
  which `PinocchioDynamics` computes from the arm model plus the **configured
  payload**. MuJoCo's own gravity compensation is switched off for the
  robot's bodies.

The MuJoCo plant keeps the true gripper bodies. Any gap between the
configured payload and the simulated gripper therefore acts on the arm, as a
wrong tool configuration does on the real robot. Attachments that aren't
part of the payload (e.g. a wrist camera) are not compensated, in sim and on
the real robot alike.

This is the general rule for any robot that uses the interface: the backend
reproduces what the robot itself does. A robot whose firmware doesn't
compensate gravity has its real backend add `state.gravity` too.

Coriolis compensation is the controller's job, using `state.coriolis`.

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
  virtual void set_goal(const VectorXd& q_goal,        // start a new segment from the current setpoint
                        std::optional<double> duration = std::nullopt) = 0;
  virtual JointSetpoint step(double dt) = 0;           // advance by dt, return the setpoint
};
class CartesianInterpolator { /* same, with Pose */ };
```

Timing:

- Every goal starts a segment with a fixed duration, set by the
  implementation (`LinearJointInterpolator(duration)`), typically one policy
  period, `1 / policy_rate`. That is today's FR3 behaviour
  (`traj_interpolation_time_fraction = 1.0`).
- The interpolator counts time only through the `dt` passed to `step()`;
  progress is `clamp(elapsed / duration, 0, 1)`. When the segment is finished,
  the setpoint holds the goal with zero velocity.
- A new segment starts at the **current setpoint**, never at the previous
  goal or the measured robot state, so the setpoint doesn't jump when a goal
  arrives before the previous segment is finished. The measured state is
  only used through `reset()`, when control (re)starts.
- The optional `duration` argument overrides the duration for that one goal.
  The FR3 backend uses it for the slow approach when control starts far from
  the first goal (`blocking_move_on_start`).

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
  virtual void set_goal(const VectorXd& q_goal,           // forwarded to the interpolator
                        std::optional<double> duration = std::nullopt) = 0;
  virtual VectorXd step(const ControllerState& state) = 0; // joint torque without gravity
};
class CartesianController { /* same, with a Pose goal in the base frame */ };
```

Joint and Cartesian are separate interfaces on purpose. Each is about ten
lines, the goal type is checked at compile time, and the backend only has to
know "is the last command a joint or a Cartesian one".

## Dynamics model (core package)

`PinocchioDynamics.h`

```cpp
struct Payload {                      // load at the flange, in the flange frame
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // about the CoM
};

class PinocchioDynamics {
 public:
  PinocchioDynamics(const std::string& model_path, bool urdf,
                    const std::string& flange_frame, const Pose& tcp_offset);
  void set_payload(const Payload& payload);
  ControllerState compute(const VectorXd& q, const VectorXd& dq, double dt);
};
```

- `compute()` does the same as libfranka's `franka::Model`, which (since
  libfranka 0.14) is itself Pinocchio: `mass` from `crba`, `gravity` from
  `computeGeneralizedGravity`, `coriolis` from `rnea(q, dq, 0) - gravity`,
  `jacobian` and `tcp_pose` from the frame at `flange * tcp_offset`
  (`LOCAL_WORLD_ALIGNED`, i.e. base frame).
- `set_payload()` adds the payload to the inertia of the flange body, as
  libfranka does with `I_total`, `m_total` and `F_x_Ctotal`. The payload
  enters the mass matrix, Coriolis and gravity.
- The model file must contain only the arm. The gripper is part of the
  payload, so that sim and real describe it the same way.
- **FR3 model file**: `assets/robots/fr3/fr3.xml`, loaded with its joint
  armature set to zero. Both of our FR3s report the same URDF through
  `getRobotModel()`. It is Franka's public `franka_description` `fr3` (v1)
  model, and its link masses, centres of mass, inertias and kinematics are
  identical to this MJCF. The MJCF only adds joint armature (rotor inertia),
  which the robot's model and `franka::Model` don't have.

## Where the implementations live

| Implementation | Package | Notes |
|---|---|---|
| `LinearJointInterpolator`, `LinearCartesianInterpolator` | core | Straight line (slerp for rotation) from the current setpoint to the goal over a fixed duration, then hold. Generalises the 7-DoF `LinearPoseTrajInterpolator.h`, which is deleted once FR3 has moved over. |
| `JointPD` | core | `tau = Kp (q_d - q) + Kd (dq_d - dq) + coriolis` |
| `OSC` | core | Operational-space control with null-space posture: `tau = Jᵀ Λ (Kp e + Kd (v_d - v)) + Nᵀ (posture term) + coriolis`, critically damped. Position and orientation are coupled through the full 6×6 task inertia `Λ`, so the TCP follows the setpoint as a clean second-order system when the model is exact. The decoupled form (separate `Λp`, `Λo`, as in the FR3 OSC and robosuite) has cross terms that slow convergence noticeably for combined translation and rotation: on an exact plant it needed 4 s instead of 1 s for a 11 cm / 13° move. |
| `PinocchioDynamics` | core | See above. |
| `SafetyStage`, `SafeJointController`, `SafeCartesianController` | core | The safety stage of `Franka.cpp` for any robot and controller, configured by `RobotConfig::safety` (`SafetyConfig`: torque limit, torque rate limit, joint-limit margin). In this order: no torque towards a joint limit closer than the margin, the rate limit per step of the nominal control period (as `franka::limitRate`, which limits per command assuming 1 ms, so the measured `dt` is not used), the clip. It keeps its own previous output instead of `tau_J_d`. `make_controller` wraps the configured controller with it, so sim and hardware run the same code. `rcs.ROBOTS` holds the values (FR3 and Panda: 87/12 Nm, 999.999 Nm/s), `apply()` sets them. `Franka` refuses async (torque) control without it; in sim it is used when set, which the FR3 scenes do. |
| `FR3JointController`, `FR3OSC` | core | The current `Franka::joint_controller` / `Franka::osc` maths, ported one to one out of the libfranka lambdas: Coriolis feed-forward, residual mass, null-space posture, joint-limit avoidance, deadband. The safety stage at the end of both loops (joint-limit guard, torque rate limit, torque clip) is the `SafetyStage` (see below), which applies to any controller. The joint-limit avoidance uses the robot's `RobotConfig::joint_limits`, passed to the constructor, instead of the values hard coded in `Franka.cpp` (which were the Panda limits). `RobotConfig` has no robot specific defaults (empty type, `dof` 0, 2×0 limits); they are set from `rcs.ROBOTS` with `rcs.ROBOTS[robot_type].apply(cfg)`, and `SimRobot` and `Franka` reject a config without them. `FR3Config` and `PandaConfig` are therefore Python classes (`rcs_fr3.hw`, `rcs_panda.hw`) that subclass the C++ `FrankaConfig` and call `apply()` in their constructor. Being in core, the sim can use them without `rcs_fr3` installed. They are separate classes rather than options on `JointPD`/`OSC`, so that `JointPD`/`OSC` stay minimal and the port can be reviewed line by line against the old code. |

## Backends

### Selecting a controller (`RobotConfig::controller`)

A controller is identified by its config, so one optional field in the
common `RobotConfig` selects it for any backend:

```python
robot_cfg.controller = common.FR3OSCConfig()            # FR3OSC
robot_cfg.controller = common.OSCConfig()               # core OSC
robot_cfg.controller = common.FR3JointControllerConfig()
robot_cfg.controller = common.JointPDConfig(kp=..., kd=...)
robot_cfg.controller = None                             # native control
```

`make_controller()` (`ControllerConfig.h`) builds it with a linear
interpolator over one control period (sim: `1 / SimConfig::frequency`,
FR3: `1 / FrankaConfig::policy_rate`) and the robot's joint limits. A goal of
the other kind is converted: with a joint controller, Cartesian goals go
through IK; with a Cartesian controller, joint goals through forward
kinematics. A configured controller needs asynchronous control (sim:
`SimConfig::async_control`, FR3: `FrankaConfig::async_control`), because
synchronous control steps until the goal is reached, which friction can
prevent under torque control.

- Sim: `SimRobot` installs the configured controller once it also has the
  dynamics. The env creators set them (`SimEnvCreator.add_robot_env`) from
  the robot's kinematic model, the same file and site as its IK, so nothing
  else needs configuring. The sim timestep is `SimConfig::timestep`
  (default 1 ms).
- FR3: without a configured controller `Franka` keeps its defaults:
  `FR3JointController` for joint goals and `FR3OSC` for Cartesian goals,
  with the gains of `FrankaConfig`.

### MuJoCo (`SimRobot`)

- New: `set_dynamics(shared_ptr<PinocchioDynamics>)`, then
  `set_joint_controller(shared_ptr<JointController>)` and/or
  `set_cartesian_controller(shared_ptr<CartesianController>)`. Without a
  controller, `SimRobot` behaves exactly as today (native MuJoCo PD).
  Installing a controller switches the arm to torque control for good and
  makes it hold its current configuration.
- With a controller installed:
  - On install, the arm actuators are switched to torque mode at runtime, the
    same way `set_config` already rewrites `gainprm`/`biasprm` (gain 1,
    bias 0). The torque is clamped to the actuator's `forcerange`/`ctrlrange`.
    MuJoCo gravity compensation (`body_gravcomp`) is switched off for the
    robot's bodies, and the backend logs that it did (see the gravity
    convention).
  - The controller runs in an ordinary `Sim` callback registered with
    `seconds_between_calls = 0`, i.e. after every integration step. It reads
    `q`/`dq` of the new state and writes `ctrl`, which the next `mj_step`
    applies. `Sim` itself doesn't change: the state comes from
    `PinocchioDynamics`, so nothing depends on MuJoCo's `qM` or `qfrc_bias`
    being current. `ctrl` is also computed once right away on install, and the
    callback's `on_restart` (reset, time going backwards) and
    `set_joints_hard()` reset the controller and recompute `ctrl`, so no step
    runs with a torque from an old state.
  - State: `q`/`dq` from the robot's joints, then
    `PinocchioDynamics::compute()`. Gravity is set in the robot base frame
    from the base orientation in the world, because arms can be mounted
    tilted (the right arm of our duo is tilted by about 52°, as fitted from
    its logged torques).
  - Payload: from the sim robot config, set to the values configured on the
    real robot. If none is configured, it defaults to the MJCF bodies
    attached after the flange (summed mass, centre of mass and inertia),
    i.e. a perfectly configured tool. The backend warns when the configured
    payload differs noticeably from those bodies, because then either the
    robot's tool configuration or the MJCF gripper is off.
  - `ctrl = step(state) + state.gravity`.
  - The controller's model (Pinocchio) is not exactly the plant (MuJoCo).
    That is intended: it plays the part of the model error on the real
    robot.
  - `set_joint_position(q)` goes to the joint controller if one is installed.
    `set_cartesian_position(pose)` goes to the Cartesian controller if one is
    installed, otherwise IK and the joint path as today. With only a
    Cartesian controller, `set_joint_position(q)` moves the TCP to the pose
    that `q` gives. Switching between the two calls `reset()` on the newly
    active controller. `kp`/`kv` in the config are rejected under torque
    control.
  - The convergence callback (`step_until_convergence`) compares the TCP pose
    instead of joint angles while a Cartesian controller is active.

### FR3 hardware (`Franka`)

- libfranka is only used for the connection: `robot.control(...)` with
  torques, the robot state, error recovery and collision behaviour.
  `franka::Model` is no longer used.
- The bodies of the two `robot.control(...)` lambdas become: take `q`/`dq`
  from `RobotState`, `PinocchioDynamics::compute()`, `controller->step()`,
  return the torque. The payload is set from `RobotState` (`I_total`,
  `m_total`, `F_x_Ctotal`) whenever it changes, as libfranka does.
  `FrankaConfig` has no tool payload to compare it with, so there is no
  warning on hardware (a possible follow-up).
- `PinocchioDynamics` is built from `kinematic_model_path` and
  `attachment_site` of `FrankaConfig` when a controller starts, so Desk-only
  uses of `Franka` don't need a model.
- By default `Franka` builds an `FR3JointController` / `FR3OSC` from
  `FrankaConfig`, so the Python API and configs are unchanged.
  `RobotConfig::controller` replaces them (see above).

### Using the FR3 controllers in sim

`FR3OSC` and `FR3JointController` live in core and only need a
`ControllerState`, so they can be passed to `SimRobot.set_*_controller()`. The
sim then runs the exact FR3 controller, including its clipping. Franka
firmware behaviour (reflexes, collision thresholds, internal torque tracking)
is **not** modelled.

## Implementation steps (each step is reviewable on its own)

1. Spec and interfaces (header only, no behaviour change).
2. Linear interpolators in core, plus pybind and unit tests.
3. `PinocchioDynamics` in core, plus pybind. Test: same mass matrix, Coriolis,
   gravity and Jacobian as MuJoCo for the FR3 model file, with the armature
   zeroed on both sides.
4. `JointPD` and `OSC` in core, plus pybind.
5. MuJoCo backend in `SimRobot`, including the payload default and warning.
   Sim tests: hold under gravity, reach a joint goal, reach a Cartesian goal,
   switch modes, configured payload different from the gripper bodies.
6. `FR3JointController` / `FR3OSC` in core. Sim test with them.
7. `Franka` uses `PinocchioDynamics` and the FR3 controllers; delete
   `LinearPoseTrajInterpolator.h`. Before that, a read-only check on the
   robot that `PinocchioDynamics` matches `franka::Model` on live states.
8. `RobotConfig::controller` selects the controller in both backends and the
   env creators; `SimConfig::timestep` defaults to 1 ms.

## Validation

Against the real FR3s on `ai2duo` (read only: the URDF from `getRobotModel()`
and logged 1 kHz control states of the right arm, recorded with the OSC on
2026-11-16):

- **Kinematics**: `PinocchioDynamics` TCP pose vs the robot's `O_T_EE` on
  40 s of logged states: within 0.0001 mm and 0.02°.
- **Dynamics**: vs `franka::Model` built from the robot's URDF, with the Desk
  payload, on 500 logged states: mass matrix within 1e-6, Coriolis within
  2e-7, Jacobian within 2e-15.
- **Gravity and payload**: `tau_J - tau_J_d` at rest (the firmware's gravity
  compensation) is reproduced to 0.05–0.23 Nm per joint by `PinocchioDynamics`
  with the Desk payload, once the gravity direction is fitted: the arm is
  mounted tilted by about 52°, and the fitted gravity is 9.94 m/s². Without
  the payload the fit is clearly worse.
- **FR3 OSC port**: open-loop replay of the logged states and 15 Hz goals
  through `FR3OSC` + `PinocchioDynamics` vs the logged commanded torque
  `tau_J_d`: correlation 0.95–0.997 per joint; with the goals delayed by the
  measured ~15 ms latency, mean absolute error 0.02–0.13 Nm on torques with
  an RMS of 0.6–1.6 Nm. Samples within 0.3 rad of a joint limit are left out,
  because the recordings ran a newer joint-limit guard than master.

The FR3 backend with the new controllers has been built but **not yet run on
the robot**.

## Open questions

- First run of step 7 on the robot.

## Decisions

- **Interpolators**: only linear ones for now. A jerk-limited Ruckig
  interpolator would be one more `JointInterpolator` and can be added later
  without interface changes.
- **Timestep**: `SimConfig::timestep` (default 1 ms) replaces the timestep
  of the scene, so the controller runs at 1 kHz as on the FR3 and gains carry
  over. `FrankaConfig::timestep` exists for symmetry and must be 1 ms, the
  period of the libfranka loop. The default `max_convergence_steps` doubled to
  1000 so that synchronous control still waits up to 1 s.
- **Linear interpolator start point**: the current setpoint (see Timing).
  The current FR3 interpolator starts each segment at the previous goal,
  which jumps when a goal arrives early; this is a small, intended behaviour
  change for FR3.
