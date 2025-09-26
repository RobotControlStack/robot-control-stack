## v0.5.0 (2025-09-26)

### Feat

- refactor robotics library ik into own extension
- **sim**: sim config support for async and realtime mode
- **install**: copy scene files into install folder
- add random object pos to task env creator
- added a configurable RandomObjectPos wrapper for flexible object placement in collector
- add async control
- add new env creator for diffpol evaluation, refactor existing one's camera creation logic
- **xarm**: include depth in xarm realsense recording
- added new envcreator for agent evaluation
- pretty xarm
- twin grasp
- added digital twin for xarm
- grasp demo
- working real setup integration
- add logic for init using either gripper or hand
- add the simple hw tilburg test script
- add an example for robot with hand
- added an option for simulating hand
- added default config for sim tilburg hand
- added a HandWrapperSim
- stubgen for SimTilburgHand
- pybind for SimTilburgHand
- added a basic compiling sim tilburg hand
- added a grasp type enum for the hand
- added mjcf xarm7
- cartesian control
- add xArm7 extension
- calibration cache for realsense
- docker compose restructure
- docker compose add-ons
- docker Added full working command wih hardware acess
- docker add optional flags for running the container with hardware access:
- docker enhance docker file user command cpu and gpu
- docker add gpu support
- docker image for rcs
- work in progress: initial working docker
- **rcs**: docker development
- Added cartesian pose support for planning This calls the RobotEnv's ik function internally.
- Added basic joint-level OMPL
- full calibration support
- **env**: allow nested space types
- **sim**: intrinsic and extrinsic as additional camera data
- mjcf support in pinoccio ik
- **ik**: added pinocchio ik
- added stub files for rcs_fr3
- added findrcs.cmake to fr3 and used rcs instead of common
- added initial cmake structure for fr3
- **sim**: allow sim gripper to have arbitrary min max vals
- **so101**: config for sim mode
- **hw**: added hw support for so101
- **ik**: add kinematics forward function
- **gripper**: adds is_grapsed and width to info dict
- **env**: made reset optional
- **envs**: support for control of multiple robots
- improve pick cube reward
- **pyproject.toml**: add digit-interface to pyproject.toml
- **examples digit_cam envs**: working on digit integration
- **hand**: enable Binary and vector action

### Fix

- mode for async joint control and move_home
- RandomObjectPos include setting orientation
- **bindings**: robot and gripper interface with init function
- tilburg hand usage of generic sized array
- remove repeated robot_cfg
- change to trajectory planning mode
- **model**: actuatorgravcomp in xarm
- **sim**: confusion of joint and actuator values
- **sim**: fixed length simrobot ids
- data class import in init
- remove tcp offset from franka
- seg fault in examples
- relative path for scences
- call proper setter function
- **sim**: tcp offset in robot mjcf
- **ik**: pinocchio ik with mjcf
- account for binary case
- change hard-coded value orders to match hardware
- consistent function name
- add SimTIlburgHand to exported stuff
- syntax consistency
- **README**: fix docker x acces command
- **sim**: tcp offset in robot mjcf
- **ik**: pinocchio ik with mjcf
- mypy errors
- pyformat and pylint changes
- make ompl path result into a list of np arrays for better compat
- **sim**: camera render image mixup
- **calibration**: april tag offset and not visible msg
- **cam**: depth scaling corrected
- **sim camera**: remove framerate for direct camera render flag
- **cmake**: added pinocchio link dep
- implicit numpy version bumping and stubfiles
- **cmake**: added pinocchio parsers
- pinocchio ik to use custom frame
- **fr3**: pybind rcs module refs
- cycle import problem
- rcs and rcs_fr3 compile
- realsense structure
- **digit**: remove property in digit config
- **franka hand**: remove slow stopping
- **cli**: remove refactor left over
- **hw camera**: recording doesnt block lock, clear buffer waits for frames
- **examples envs**: make digit_cam work with hardware instead of simulation
- **envs**: 
- **hand**: fix grasp mechanism after refactoring
- **env creators**: gripper config types
- **examples**: import of hand example
- remaining qarts renamed
- imports for renamed rcs package
- **env**: fix some of the requersted changes
- **hand**: make default value in TilburgHandControl class instead of HandControl class
- **hand**: add get_pos_vector and set_pos_vector to Handcontol class

### Refactor

- rename ik into kinematics and inverse
- rename set and get parameters to config
- removed get tcp offset from mjcf utility
- **envs**: renamed binary hand key to gripper
- proper scene configuration
- added mjcf and kinematics path to robot config
- scene and kinematic file usage
- introduce close method for cleanup
- **env**: gripper wrapper non private gripper
- calibration with list instead of queue
- **camera**: calibration interface change
- move xtrinsics into calibration
- move examples into top level
- compatility with current master
- **ik**: abstract ik class with rl subclass
- **fr3**: moved stubs to correct location
- rename extensions with rcs prefix
- rcs headers in own include folder, header copy and fr3 rcs renaming
- build shared librcs
- removed hw module from bindings and python code
- seperate hardware extensions
- **fr3**: moved env creators and utils for fr3 module
- **fr3**: moved envs in fr3 module
- fr3 module with desk in it
- moved realsense into its own module
- **sim**: generalize gripper to use a single joint
- removed lab specific wrappers
- **sim**: gui loop to use simple frame rate
- moved frame rate and improved time tacking
- moved sim into own folder
- Remove private scenes, add public assets (#182)
- **camera**: hardware camera with composition and digit as camera
- **camera**: config and camera set
- replace pydantic with dataclasses
- **digit**: moved digit to camera folder
- **gym sim**: removed fr3 from sim wrapper
- **gripper**: id fill method and gripper with id
- **robot ids**: move ids to config
- **robot ids**: ids must match in config
- mujoco attributes in config
- stubs and ik solver rename to fit rcs package
- renamed imports and all other occurrences rcsss -> rcs
- renamed rcsss python folder to rcs
- renamed bindings file to rcs.cpp
- traj interpolator in same style
- Flexible dof and robot meta config
- rename FR3Env to RobotEnv
- renaming of env creator classes
- rename rcs sim classes in python
- rename sim robot classes pybind
- **cpp**: sim independent of robot type
- removed unused robotwithgripper class
- rename motion generator
- rename quart to quat
- **env**: factory functions into env creators
- **env**: renamed factories to creators
- rename resource manager to context manager
- remove unused n robots class
- removed unsued recordings
- **control**: remove app code
- **env**: sim specific gripper func into own wrapper
- **hand**: previous renaming in code
- **hand**: rename to remove control in name
- **hand**: python protocol interface
- **examples**: pyformat, pylint and pytest
- **examples**: 
- **envs**: fix pyformat, pylint and pytest
- **envs, hand, examples**: address the requested changes
- **hand**: beteter handling of default grasp value and add tilburg-hand library to requieremnts_dev.txt
- **hand**: fix pyformat, pylint and pytest
- **hand**: fix pyformat, pylint and pytest

## v0.4.0 (2025-05-12)

### Feat

- **sim camera**: allow camera render on demand
- **sim fr3**: make convergence callback registration optional
- **env**: camera robot
- **grasp_lab_demo**: added second robot cam pose setter
- **grasp_demo_lab**: unified cube pose randomizer and separated the pose setter for the second robot
- **grasp**: pickup script for grasp added
- **sim**: separated grasp_demo script from the grasp_demo_lab script
- **wrapper recorder**: hdf5 gzip compression
- frame rate logger
- **wrapper**: added hdf5 recorder wrapper
- **wrappers**: storage wrapper delays action
- **hw gripper**: bool state and is moving
- **hw gripper**: async control for hardware gripper
- **camera**: added rate limiter
- added async for teleop
- added webcam live viewer and vla executer
- updated vive teleop to use controls and data recording
- added first osc implementation
- **env**: reward and success for simple pick up env
- **env**: random cube pos wrapper
- **sim**: sim wrapper env
- **gym env**: register envs in gym
- **hw camera**: record frames as mp4 video
- **factories**: added collision guard to factories and examples
- **cli**: info command to get current robot and gripper state
- **hw/camera**: option to wait for frames in buffer
- **sim/camera**: optional conversion to pyhsical units for depth camera in sim
- **pose**: read out total quat angle and set angle

### Fix

- **fr3 desk**: error when circle is not in event dict
- **env**: type assertion and improved tests
- wrong automatic import and logger
- type linting issues
- **env**: moved close of control thread to hw env
- **sim gui**: increases max geoms in render gui
- **hw fr3**: default register convergence callback
- **py lint**: type annotations and imports
- **sim**: allow string path in tcp offset getter
- **control**: joint reset also sets controllers
- **grasp_demo_lab**: removed side camera feature and refactored
- **sim**: random cube placement in lab scene fixed
- **merge**: merge commit
- **desk**: remove guiding mode in move home
- **hw camera**: removed double video saving
- **desk**: cache token in case of program crash
- **env gripper**: remove same action ignore
- **hw gripper**: exception handling for franka hand, improved roboustness
- remove slow unnecessary flatten
- lock usage in camera
- async reset
- relative move move high freqency filter
- **hw**: joint controller torque limit
- **rel env**: relative limiting for origin set
- **vive**: collision guard
- lock usgage, controllers, remove unused code
- double reset segfault / fp error (Merge pull request #163 from utn-mi/krack/fix_double_reset)
- linting errors in examples and tests
- numpy < 2 compatibility
- **env**: binary gripper observation
- **sim**: reset sim
- **collision guard**: back compatibility truncate episode
- **collision guard**: hard sim update and identity action
- **collision guard**: updated gui visualization
- **factories**: default relative_to and max_realtive_movement type
- **vive**: the collision guard is no longer a bool, so updated it to default None
- **realsense**: properly stop camera
- **examples**: ik usage in fr3
- **envs/colguard**: order of sim env, gym interface and typing
- **envs/gripper**: avoid overwrite of collision
- **sim/gripper**: fixed is_moving and simplified callback schema
- **sim**: corrected reset observation
- **envs**: stop camera thread in camera env with close method
- support numpy versions < 2.0
- rel env last step default and gripper wrapper correct last step detection
- dot env in cli, panic activates guiding mode
- **example, rel env**: added last step relative to examples and fixed rel env
- **rel env**: improved relative limiting, avoiding clip

### Refactor

- **cli**: remove remaing config functions
- **control**: remove vive and recorder app code
- **sim envs**: simple and lab pick up share more code
- **lab env**: flexible camera robot joint position
- **envs**: tcp extraction before env creation
- **envs**: move default configs to utils
- **grasp_demo_lab**: resolved pylint issues
- **grasp_demo**: refactored to correct linting issues
- **grasp_demo**: refactored to correct linting issues
- **grasp_demo**: refactored to correct linting issues
- **wrappers**: fixed gif and video generation for HDF5 data
- **recorder wrapper**: moved gif generation into own function
- **storage wrapper**: flag removed to simplify interface
- **hw**: zero torque guiding integrated into control thread flow
- **hw**: guiding mode configure DOFs
- **hw control**: async control in config
- proper rate limiter in vive
- **hw**: moved stop async control thread to reset
- **hw camera**: video recording in storage wrapper
- video frame is written every step
- **vive**: more conservative lock usage
- **factories**: same mjcf scene for sim collision guard
- **cameraset**: close method in cameraset interface
- **env/gripper**: better observation difference between binary and non binary gripper
- **pose,rel env**: limitation of rot and trans moved to cpp

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
