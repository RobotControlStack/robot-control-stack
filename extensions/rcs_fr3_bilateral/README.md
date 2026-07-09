# rcs_fr3_bilateral

Boilerplate for bilateral FR3 teleoperation in RCS.

The extension wraps two `rcs_fr3` `Franka` objects, one leader and one follower. It starts two worker threads:

- the leader worker samples leader joint state and prepares follower targets;
- the follower worker samples follower state, captures external joint torques, and sends follower joint targets.

The current implementation performs the following:
- Follower's joint positions are set using `rcs_fr3`'s `set_joint_position` and `joint_controller`, which currently has a fixed `Kp` and `Kd` gains.
- Leader side is simply left as a gravity compensation controller, and the follower's `tau_ext` is applied on top of it, realized as a `set_joint_torque` and `torque_controller`, i.e. the actual commanded torque is simply the `tau_ext` of the follower. The gains of the haptic feedback can be adjusted using `haptic_feedback_gain` under `BilateralFrankaConfig`.

The structure follows the Franka ROS 2 teleoperation example: follower joint impedance tracks the leader, and the leader receives the follower's estimated external joint torques as force feedback.

## Example

Start in gravity-only sanity-check mode:

```bash
python example/bilateral_env.py --leader-ip 192.168.101.1 --follower-ip 192.168.102.1 --mode gravity_only
```

Switch to the active follower-tracking path:

```bash
python example/bilateral_env.py --leader-ip 192.168.101.1 --follower-ip 192.168.102.1 --mode bilateral
```

Only perform joint-to-joint mapping without haptics:
```bash
python example/bilateral_env.py --leader-ip 192.168.101.1 --follower-ip 192.168.102.1 --mode bilateral --disable-leader-haptics
```