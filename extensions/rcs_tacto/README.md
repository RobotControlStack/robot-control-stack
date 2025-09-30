# TACTO integration for RCS
This package can be installed by running `pip install -e extensions/rcs_tacto` from the RCS repository root.

An example on how to create the environment is found in `{REPO_ROOT}/examples/grasp_digit_demo.py`. 

Particularly, take a look at `FR3TactoSimplePickUpSimEnvCreator` to understand how Tacto is inserted into the RCS stack.

Note that it is rather tricky to get the correct contact simulation settings to allow for a robust grasping of objects, so when using it, you will need to play around with the simulation settings. We recommend taking a look at the corresponding [MuJoCo documentation](https://mujoco.readthedocs.io/en/stable/computation/index.html) for more tips.