# Franka Teleoperation

## Teleoperation with Meta Quest 3
Teleoperate your robot (optinally dual arm) with the Meta Quest

### How does it work?
In the script [`franka.py`](franka.py) we use the [IRIS platform](https://intuitive-robots.github.io/iris-project-page/index.html) to get controller poses from the meta quest.
With the relative space wrapper and the relative to configured origin setting theses poses are then apply to the robot in a delta fashion whenever the trigger button is pressed.
The buttons are used to start and stop data recording with the [`StorageWrapper`](robot-control-stack/python/rcs/envs/storage_wrapper.py).

### Installation
[Install RCS](https://robotcontrolstack.org/getting_started/index.html) and the [FR3 extension](https://robotcontrolstack.org/extensions/rcs_fr3.html) (the script is writte for the FR3 as example but can be easily adapted for other robots).
Install the IRIS APK on your quest following [these instructions](https://github.com/intuitive-robots/IRIS-Meta-Quest3) use the apk released [here](https://github.com/RobotControlStack/IRIS-Meta-Quest3/releases/tag/rcsv1) to ensure compatiblity.
Finally, install [SimPub](https://github.com/intuitive-robots/SimPublisher) the IRIS python client by

```shell
pip install -r requirements.txt
git clone https://github.com/intuitive-robots/SimPublisher.git
cd SimPublisher
git checkout a0ebdbfba86f58aaa6360eb3b9da7b74a9c9b1dd
pip install -ve .
```

### Configuration

#### Teleoperating in sim

1. go to [`franka.py`](quest_iris_dual_arm.py) and set `ROBOT_INSTANCE = RobotPlatform.SIMULATION`

#### Teleoperating a real robot
Note that dual arm is only supported for a aloha like setup where the robot face each other (for more advanced setups you need to change the transformation between the robots yourself).
1. put your robots into FCI mode
2. go to [`franka.py`](quest_iris_dual_arm.py), set `ROBOT_INSTANCE = RobotPlatform.HARDWARE` and set your IP addresses of your robots. Remove the left robot if you only have one.


### Running
1. make sure your computer and quest is in the same subnetwork and they can ping each other.
2. start IRIS meta quest app on your quest (it should be located in the Library under "Unkown Sources" after installation)
3. run the [`quest_align_frame.py`](quest_align_frame.py) script once. Navigate to the link printed on the top likly [http://127.0.0.1:7000](http://127.0.0.1:7000).
    - click scan
    - your meta quest should show up
    - click change name and type "RCSNode" and click ok
    - the script should now print a bunch of numbers (the controller poses)
4. put on your quest (dont remove it until done with teleop, otherwise the axis might change and you need to recalibrate), you should see a white ball with coordinate axis somewhere in your room (red is x, green is y and blue is z)
5. use the right controller to change the orientation of the coordinate axis to fit your right robot (for franka: x front, y left, z up)
6. click the "teleportation scene" button on the still open website
7. cancel the script
8. start the teleoperation script [`franka.py`](quest_iris_dual_arm.py) and enjoy.


## Teleoperation with Franka GELLO Duo
Teleoperate your Franka Duo using the [Franka GELLO Duo](https://franka.de/de-de/product-prototypes).
Install dependencies via
```shell
pip install -r requirements.txt
```
and make sure the `GelloConfig` is commented in and the `QuestConfig` is commented out and adapt your USB IDs to it.