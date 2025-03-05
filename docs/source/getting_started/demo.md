# Demo

python/examples contain multiple example codes that can be executed upon first installation

1. A simple demonstration which moves the tcp forward 1cm in the x direction, closes the gripper, moves back 1cm and opens the gripper.
This cycle is repeated multiple times.

```
python python/examples/env_cartesian_control.py
```

2. A simple demonstration where a random action is sampled from the joint space and executed on the robot.

``` 
python python/examples/env_joint_control.py
```

3. A simple pickup experiment showing Franka picking up a randomly placed cube.

``` 
python python/examples/grasp_demo.py
```