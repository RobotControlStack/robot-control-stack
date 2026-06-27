# rcs_sb3

- Entry: `run_test.py`\
- training: `run_test.py --mode train --num-envs <n> --total-timesteps <n> --n-steps <n> --batch-size <n>`
    - setting `--num-envs 0` runs it on the main thread for breakpointing
- inference: `run_test.py --mode infer --infer-steps 100`
- Change the paths of the variables `_BOX_XML`, `_TAXIM_GRIPPER_XML` and `_NORM2TEX_DIR` accordingly
- To enable textures, you need to pass the flag `--with-textures` with the generated files (`_uv,_normal,_color`) in `norm2tex/grasp_assets/box/assets`; without the flag, it should just run the "smooth" version.

Installation:
```
git clone git@github.com:RobotControlStack/robot-control-stack.git
<cd to robot-control-stack>
git checkout jin/sb3

# Install RCS
conda create -n rcs12 python=3.12
conda activate rcs12
conda install -c conda-forge urdfdom urdfdom_headers glfw

# or sudo apt install $(cat debian_deps.txt)
pip install 'pip>=25.1'
pip install --group build_deps
sudo apt install liburdfdom-dev

# install rcs
pip install -ve .

## Install the dependencies for sb3
# clone and install norm2tex main branch
git clone git@github.com:utn-air/norm2tex.git
<cd to norm2tex>
pip install -e . 

# clone and install mujoco-taxim norm2tex branch
git clone git@github.com:utn-air/mujoco-taxim.git
<cd to mujoco-taxim>
git checkout norm2tex
pip install -e .

# install rcs_taxim 
<cd to robot-control-stack>
pip install -e extensions/rcs_taxim

# finally install rcs-sb3
<cd to robot-control-stack>
pip install -e extensions/rcs_sb3

# now try running the run_test.py
python run_test.py --gui --visualize-taxim --steps 1000

``

## To-do
- Add json file interpreter for feeding geom-texture png files into mujoco-taxim for easier management (Done)
- Make the textures for the cube (Done, for now)
- Enable norm2tex (Done)
- Investigate parallelization (Done)
- Add wrapper for truncation condition
- Add wrapper for reward calculation