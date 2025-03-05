# Wrappers

```
Warning: 
This content has not yet been proof read!
```

[Wrappers](https://www.gymlibrary.dev/api/wrappers/) are a convenient way to modify an existing environment without having to alter the underlying code directly.
RCS offers the following wrappers that add functionality in a layered manner.

1. FR3Env

    This is the basic Joint Gym Environment for Franka Research 3.

2. FR3Sim

    This wraps the basic FR3Env with the MuJoCo simulation.

3. CameraSetWrapper

    This is an observation wrapper, that takes the observation from its parent environment and add rgbd information.

4. GripperWrapper

    This is an observation wrapper, that takes the observation from its parent environment and add gripper_state(whether open or close) information.

5. CollisionGuard

    This wrapper is used for safety from collisions. Before executing the action, this collision guard executes the action in simulation in a sub-collision environment to ensure there is no collision, after that the action will be executed on the parent environment. In case of collision, the user can decide whether to truncate the episode.

6. RelativeActionSpace

    This wrapper clips the actions based on pre-defined maximum limits and steps the clipped actions on the parent environment.