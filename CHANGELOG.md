## v0.3.1 (2024-10-02)

### Fix

- **ik**: ik optional bug and fr3 example

## v0.3.0 (2024-10-02)

### Feat

- **sim**: Interactive sim viewer in separate process
- **gui**: refactor with base class
- **gui**: GuiServer & GuiClient class skeletons
- **gui**: add mujoco ui library

### Fix

- **camera env**: missing depth data when depth enabled
- remove doubly registered collision callback

### Refactor

- **vive**: using factory functions to create envs
- **sim**: simulation to use refactored ik
- **ik**: moved rl ik into common, removed rl deps in hw

## v0.2.2 (2024-10-01)

### Feat

- **rcsss/envs/base.py**: adding the depth data to the CameraSetWrapper
- **src/sim/camera.cpp**: added depth image data to the frameset

### Fix

- **camera env**: rgb+depth mode
- **sim camera**: using defined keys in env not id
- **python/rcsss/_core/sim.pyi**: updated missing pyi file

### Refactor

- **env camera**: fix linting, remove dict merge, assert available depth
- **camera env**: correct handling of rgb and optional depth
- **camera**: improved variable naming
- **rcsss/camera/sim.py**: asserting that rgb and depth info come from the same camera
- **python/rcsss/camera/sim.py**: pylint fixes

## v0.2.1 (2024-09-13)

### Fix

- **examples**: desk import and max movement

## v0.2.0 (2024-09-13)

### Feat

- added non-env rcsss grasp example
- **env**: parameterizable max movement in relative env
- Added keyboard-based robot teleoperation
- typed factories and tests for gym envs (#81)
- **sim**: finger in gripper collisions and ignored geoms (#117)
- add ring buffer for hardware cams
- **env**: avoid execution of zero action on the robot
- gripper in teleoperation
- cart. control mode wrapping in collision guard env
- collision guard forwards joint ik values
- tcp offset with constant function in cpp
- **cpp::hw**: tcp offset and rl inverse kinematics
- **rcsss**: added live plotter for robot poses
- **envs**: added set_origin method to relative space
- random movement example on real robot
- fci context manager supports lock
- binary gripper wrapper flag
- **cli**: passive fci command that takes and maintains control
- **sim**: added is_coverged to stub files, added to gym env info dict
- **sim**: is_converged method and max_convergence_steps to avoid stucking
- bump my version release management tool
- Pose constructors with identity rotation/translation
- **env**: added option to avoid checking collision when moving home in CollisionGuard
- **env**: added collisionguard env
- added inital observation to relative action space wrapper
- Franka Hand in Mujoco Simulation
- Change of Abstract Robot and Gripper Interfaces
- **env**: Implemented quaternion actions
- **py::cam**: per camera config in python
- **cpp::cam**: added multi cam config support
- **cpp::camera**: using option from renderer
- **cpp::cam**: added camera init and render code
- **envs**: Gym space annotated observation and action types
- **cpp::pose**: added xyzrpy fun to pose
- new env wrapper classes and env cleanup
- **cpp::cam**: added scene to render callback
- **cam**: added timestamp to simcamset
- **cam**: added bindings for sim camera set
- **cam**: Base camera sim impl. in cpp

### Fix

- broken desk path
- refactored fr3 desk paths
- **camera**: check for running thread in hardware cameras
- cpp linting errors
- **examples**: resource manager and tcp offset
- **examples**: env_joint_control import name
- **env**: read gripper state instead of cache
- **examples**: credentials usage and contex manager
- added missing dependency for examples
- **cpp::sim**: respect col vs row major in cartesian readout in mjc
- **factories**: camera in coll guard, relative env usage
- **camera**: Create camera configuration before calling the base class constructor
- **camera**: Store the number of elements in the ring buffer to handle cases when the buffer is not full yet
- **vive**: Removed nested with statement and fixed formatting
- Prevent random restarts and set precision in RL IK
- **env**: Set threshold for comparing actions with np.allclose
- **vive**: Fixed calculation of relative rotation and enabled live plot with a non-blocking socket
- **env**: import of sim
- sim gripper collision in state (#118)
- **env**: access non existing var in grasping env
- **cpp::sim**: mj quat format
- **env**: added missing rpy class to array conversion
- **env**: computation of trpy action in relative environment
- bump to latest pybind version
- numpy version fixed to 2.0
- **pose**: fixed rpy output
- **env**: binary gripper return value
- **config**: added camera type to sample config
- type hinting hw camera set
- transpose tcp offset
- style and linting, FrankaHandTCPOffset returning matrix
- **vive**: gil release and tcp offset
- **vive**: fixed transforms and added live plotting code
- **common**: use extrinsic Euler angles in xyz format
- **sim**: calculation of the TCP offset
- **rel env**: order of diff pose multiplication
- **vive**: button press logic and pose calculation
- mypy lint issues
- **sim**: convergence detection (#106)
- **cpp**: removed isapprox for isclose comparisons (#105)
- **sim**: max convergence steps logic (#104)
- pose tests to use pose_matrix arg
- quaternion typo
- **pose**: calculation of pose concatination
- **pose**: inversion
- **docs**: Added missing deps
- updated models git submodule version
- etils version for python 3.10
- resolved mypy typing issues
- wrong ellipsis in stub files
- cpp linting
- **env**: removed usage of depricated .env
- RelativeActionSpace Wrapper can have arbitrary wrap order
- **envs**: copy observations and actions
- removed wrong basecameraconfig imports
- quaterion normalization in pose constructor
- **stub**: numpy shape generation in stub file for dynamic sized vectors
- relative env fixed max movement
- config generation of multi cam sim config
- **cpp::cam**: import and deconstructor order
- **cpp::camera**: Return types of framesets, pass of camera name to poll fun
- **cpp::camera**: changed storage types and fixed deconstructor
- **camera**: hand over from cpp to python
- **envs**: type checking with existance of keys in dict
- **cam**: stub files, bindings and sim exports

### Refactor

- method to load credentials and linting fixes
- fixed import paths for control folder
- hw franaka hand use reset method for init
- env use robot reset method
- **examples**: renamed env example files
- **env**: moved examples into own folder #111 and simplified them
- **envs**: extracted two example methods in hw env
- **vive**: format and refactored hw and sim in two methods

## v0.1.0 (2024-06-28)

### Feat

- **env**: CameraSet Gym Env
- **cli**: testing cli for realsense
- **camera**: added draft impl. for realsense
- **env**: CameraSet Gym Env
- **cli**: testing cli for realsense
- **camera**: added draft impl. for realsense
- **gitlab-ci**: added gitlab ci and gitlab runner docker file
- **unittests**: inital setup for unittests - initial directory setup for unittests - wrote a dummy test for pose Interpolation method
- **unittests**: inital setup for unittests - initial directory setup for unittests - wrote a dummy test for pose Interpolation method
- **unittests**: inital setup for unittests - initial directory setup for unittests - wrote a dummy test for pose Interpolation method
- **cpp::Pose**: added new rpy as vector constructor
- **envs**: added sim and hw envs
- **py::record**: tj recording with FR3 buttons
- **bindings**: moved franka exceptions into own submodule
- **bindings**: added franka exception translation
- **bindings**: covariant return type functions in bindings
- **cpp**: extended pose and rpy with new methods
- **cpp**: added pickle support for pose and rpy cpp types
- **config**: cli tool to generate sample config
- **rcsss**: added kinect camera support
- **rcsss**: auto generation of stub files
- **CI pipeline**: python lint, format and build, apt cache
- **hw**: added config arg in the initializer
- **hw**: Implemented gripper interface
- **hw**: Implemented robot interface
- **common**: Improved robot interface
- **common**: added rotation matrix to rpy struct
- **common**: Added python bindings
- **cpp::common**: Added N robots class
- **cpp**: robot and gripper interfaces, eigen definition in common
- **common**: added python bindings for pose

### Fix

- **gym**: reset method typing and _get_obs method
- **camera**: linter issues and gym env
- **realsense**: working draft
- **gym**: reset method typing and _get_obs method
- **camera**: linter issues and gym env
- **pipeline**: merged build and stubgen
- **core arch**: removed set_parameters in base robot class
- **realsense**: working draft
- **pose**: fixed is_close function, see #52
- **envs**: fixed type hinting with type alias
- **types**: mypy env type checking
- **stubs**: wrong vector stubs
- **desk**: config, take control and cli
- **bindings**: readded pybind trampoline
- **desk**: Bugs found when tested on the hardware
- **bindings**: Using default unique_ptr as holder for configs
- **py:camera**: convert c types to np array
- **CI**: mypy type check fixes
- **pybind**: Add config and state struct constructors
- **stubs**: added latest stub files
- **lint**: change stubgen to match mypy expectations
- **CI**: mypy install types
- **record.py**: mypy type complains, new rcs interface
- **py**: linting and formatting errors
- **py::desk**: linter errors
- **rcsss**: python module import
- **common**: added missing unique_ptr in n robots
- **cmake**: Adapted and fixed cmakelists files
- **hw::FR3**: Fixed inertia datatype and set para func
