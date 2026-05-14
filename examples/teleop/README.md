# Franka Teleoperation

## Teleoperation with Meta Quest 3
Teleoperate your robot (optinally dual arm) with the Meta Quest

### How does it work?
In the script [`franka.py`](franka.py) we use the [IRIS platform](https://intuitive-robots.github.io/iris-project-page/index.html) to get controller poses from the meta quest.
With the relative space wrapper and the relative to configured origin setting theses poses are then apply to the robot in a delta fashion whenever the trigger button is pressed.
The buttons are used to start and stop data recording with the [`StorageWrapper`](robot-control-stack/python/rcs/envs/storage_wrapper.py).

### Installation
[Install RCS](https://robotcontrolstack.org/getting_started/index.html) and the [FR3 extension](https://robotcontrolstack.org/extensions/rcs_fr3.html) (the script is writte for the FR3 as example but can be easily adapted for other robots).
Install the IRIS APK on your quest following [these instructions](https://github.com/intuitive-robots/IRIS-Meta-Quest3) use the apk released [here](https://github.com/RobotControlStack/IRIS-Meta-Quest3/actions/runs/25190284304) to ensure compatibility.
Finally, install [SimPub](https://github.com/intuitive-robots/SimPublisher) the IRIS python client by

```shell
pip install -r requirements.txt
git clone https://github.com/RobotControlStack/SimPublisher.git
cd SimPublisher
pip install -ve .
```

### Configuration

#### Teleoperating in sim

1. go to [`franka.py`](franka.py) and set `ROBOT_INSTANCE = RobotPlatform.SIMULATION`

#### Teleoperating a real robot
Note that dual arm is only supported for a aloha like setup where the robot face each other (for more advanced setups you need to change the transformation between the robots yourself).
1. put your robots into FCI mode
2. go to [`franka.py`](franka.py), set `ROBOT_INSTANCE = RobotPlatform.HARDWARE` and set your IP addresses of your robots. Remove the left robot if you only have one.


### Running
1. Make sure your computer and quest is in the same subnetwork and they can ping each other.
2. Check your local machine IP using `ip addr` in the terminal and note the IP address of the network interface in the same subnet as your Quest (e.g. `192.168.1.x` or `10.42.0.1`). Replace it in `examples/teleop/franka.py`: `MQ3_ADDR = "<your_pc_ip>"`
3. Start IRIS meta quest app on your quest (it should be located in the Library under "Unknown Sources" after installation)
4. Run the [`quest_align_frame.py`](quest_align_frame.py) script once. Navigate to the link printed on the top likely [http://127.0.0.1:5000](http://127.0.0.1:5000).
    - click scan
    - your meta quest should show up
    - click change name and type "RCSNode" and click ok (this needs to be done only once)
    - restart both the app and the script
    - the script should now print a bunch of numbers (the controller poses) which means the connection is working
5. Put on your quest (don't remove it until done with teleop, otherwise the axis might change and you need to recalibrate), you should see a white ball with coordinate axis somewhere in your room (red is x, green is y and blue is z)
6. Use the right controller to change the orientation of the coordinate axis to fit your right robot (for franka: x front, y left, z up)
7. Click the "teleportation scene" button on the still open website
8. Cancel the script
9. Start the teleoperation script [`franka.py`](franka.py) and enjoy.


## Teleoperation with Franka GELLO Duo
Teleoperate your Franka Duo using the [Franka GELLO Duo](https://franka.de/de-de/product-prototypes).
Install dependencies via
```shell
pip install -r requirements.txt
```
and make sure the `GelloConfig` is commented in and the `QuestConfig` is commented out and adapt your USB IDs to it.