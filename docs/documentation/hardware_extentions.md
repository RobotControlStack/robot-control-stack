# Hardware Extensions

RCS supports integration with various hardware platforms via dedicated **hardware extensions**.  
These allow you to run the same RCS APIs on **real robots and sensors** without modifying your main codebase.

For example, the **Franka Emika Research 3 (FR3)** robot is supported via the [`rcs_fr3`](extensions/rcs_fr3) extension.

All natively supported extensions are located in the [`extensions`](extensions) directory.

---

## 1. Installing Hardware Extensions

To enable hardware usage in RCS, install the desired hardware extension via `pip`:

    pip install -ve extensions/<extension_name>

For example, to install the FR3 extension:

    pip install -ve extensions/rcs_fr3

>  **Tip:** Each extension may have its own setup requirements. See the specific extension's section below for additional configuration steps.

---

## 2. Switching Between Simulation and Hardware

After installing the required extension, you can switch your RCS instance from simulation to hardware by setting:

    from rcs.common import RobotPlatform

    ROBOT_INSTANCE = RobotPlatform.SIMULATION   # Default: Simulation
    # ROBOT_INSTANCE = RobotPlatform.HARDWARE   # Uncomment to use real hardware

---

## 3. Command-Line Interfaces (CLI)

Some extensions provide CLI commands to interact with hardware without writing Python code.

For example:

    python -m rcs_fr3 --help
    python -m rcs_realsense --help

These commands allow you to perform basic operations directly from your terminal.

---

## 4. FR3 Hardware Extension

The **RCS FR3** extension enables control of the **Franka Emika Research 3** robot via RCS.

### 4.1 Additional Configuration

1. Create a `.env` file in your working directory with your FR3 Desk credentials:

       DESK_USERNAME=your_username
       DESK_PASSWORD=your_password

2. Set your FR3’s IP address in your script:

       ROBOT_IP = "192.168.0.1"  # Replace with your robot's IP

---

### 4.2 Usage Example

    import numpy as np
    import rcs
    import rcs_fr3
    from rcs_fr3._core import hw
    from rcs_fr3.desk import FCI, Desk, load_creds_fr3_desk
    from rcs_fr3.config import FR3Config, IKSolver
    from rcs.common import Pose, FrankaHandTCPOffset, RobotPlatform

    ROBOT_IP = "192.168.0.1"
    ROBOT_INSTANCE = RobotPlatform.HARDWARE

    # Load credentials
    user, pw = load_creds_fr3_desk()

    # Connect to the robot
    with FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False):
        urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
        ik = rcs.common.RL(str(urdf_path))

        # Initialize robot
        robot = hw.FR3(ROBOT_IP, ik)
        robot_cfg = FR3Config()
        robot_cfg.tcp_offset = Pose(FrankaHandTCPOffset())
        robot_cfg.ik_solver = IKSolver.rcs_ik
        robot.set_parameters(robot_cfg)

        # Configure gripper
        gripper_cfg = hw.FHConfig()
        gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
        gripper_cfg.speed = 0.1
        gripper_cfg.force = 30
        gripper = hw.FrankaHand(ROBOT_IP, gripper_cfg)

        # Move and grasp
        robot.set_cartesian_position(
            robot.get_cartesian_position() * Pose(translation=np.array([0.05, 0, 0]))
        )
        gripper.grasp()

---

### 4.3 CLI for FR3

The FR3 extension also defines useful CLI commands for controlling the robot without the Desk website:

    python -m rcs_fr3 --help

---

## 5. Additional Resources

- **Examples:** See the [`examples`](../../examples/) folder for more usage samples.
-  **Franka Desk Documentation:** Refer to the FR3 manufacturer’s manual for setup and safety.
