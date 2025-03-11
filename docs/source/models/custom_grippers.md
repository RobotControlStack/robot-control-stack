# Custom Grippers

Custom grippers can be used with RCS by following the instructions below.
## How to add a custom gripper
An example custom gripper is available for reference [here](https://gitos.rrze.fau.de/utn-machine-intelligence/lab/models/-/tree/main/grippers/digit_hand/obj?ref_type=heads) **(only if you have access to the RCS models repo)**

The adjustment to fr3_0.xml and fr3_common.xml can be found [here](https://gitos.rrze.fau.de/utn-machine-intelligence/lab/models/-/tree/main/scenes/fr3_simple_pick_up_digit_hand?ref_type=heads) **(only if you have access to the RCS models repo)**

### Step1: export obj files
If the gripper is downloaded from somewhere convert it to .obj file.
Make sure that the x, y, z axis of the default gripper and the custom gripper are the same. In case they are different, later the slide joint axis might have to be adjusted. This can be done by visualing the coordinate axis, by dragging the obj files in onshape.
### Step2: convert obj to mjcf
**Note:** First ensure that the file_names dont have space in their names

**e.g** rename <Digit Stub Full.mtl> to <Digit_Stub_Full.mtl> in three places(obj file, mtl file, mtl file reference inside the obj file)

A python package for converting obj files to mjcf files is available [here](https://pypi.org/project/obj2mjcf/0.0.3/).
After installing it, use the following command to do the required conversion.



```shell
obj2mjcf --obj-dir <obj_dir_name> --save-mjcf
```


### Step3: verify in mujoco viewer
With the above two steps, the conversion should be successful, you should be able to view the standalone grippers in mujco with the following commands

```shell
python -m mujoco.viewer
```


after opening the mujoco viewer you can drag the converted scene and visualise it.

### Step4: Adding the gripper to the lab or fr3_empty_world scenes
    The following steps need to be followed:
        1. in fr3_common, the mesh should be renamed to the new gripper
        2. in fr3_0 and fr3_1 and all the robots, the fingers’ geom must be adjusted for material and if there are name changes
        3. physics parameters should be adjusted empirically..
           e.g. new collision meshes specific to the gripper added, friction parameters adjustment

## step5: update the tcp offset
  add the correct tcp_offset value to the following to fr3_common.xml


    ```
    <custom>
		<numeric
			name="tcp_offset"
			data="0 0 0.0466"
	    />
    </custom>
    ```
