# :ribbon: Autopilots :ribbon:

## Software in the Loop (SITL)

<p>To run SITL either: <br>

 ```
cd ardupilot/ArduCopter
sim_vehicles.py -w --map --console
```
<p>Or you can specify from the home directory like this</p>

```
sim_vehicles.py -v ArduCopter
```

<p>Sometimes, you can exit out of all the software incorrectly, so there are still connections to your local ports. This will not allow you to run any other simulations. The error could look like: </p>

```
MAV> link 1 down
```

<p>You need to close the previous instances of SITL or MAVProxy by running:</p>

```
pkill -f arducopter
pkill -f mavproxy.py
```

## Gazebo

### Simulating the virtual environment

<p>You can edit how the Gazebo simulated world looks like by either making a script, building an environment, or editing an existing world. The script is written in XML which is like HTML. XML defines back-end descriptions and functions whereas HTML defines UI. The individual worlds and model scripts are written .sdf format. There are other dependencies to also include, such as referenced images and material scripts from Gazebo.</p>

<p>SDF Documentation: http://sdformat.org/spec?ver=1.8&elem=sdf</p>


## IQ Sim Tutorials

<p>In one terminal, run:</p>

```
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
```

<p>In the second terminal run:</p>

```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

## Notes

<p>As of now, my .bashrc file contains a source for ROS 1 (Noetic). Later, if I want to develop with ROS 2 just be aware to edit the bashrc file and manually source ROS 1 instead (vice versa).</p>

To manually source:

```
source /opt/ros/noetic/setup.bash
```

### Links & References
- Get emojis :tulip: in .md: https://gist.github.com/rxaviers/7360908
- Drone Dojo Programming Course: https://github.com/dronedojo/droneProgrammingCourse
- SITL on Windows 10 using WSL: https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html#sitl-on-windows-wsl
- ArduPilot DroneKit Tutorial: https://ardupilot.org/dev/docs/droneapi-tutorial.html
- DroneKit Repository: https://github.com/dronekit/dronekit-python/blob/master/README.md
- ArduPilot Introduction: https://ardupilot.org/dev/docs/learning-ardupilot-introduction.html
- IQ Tutorials: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md
- IQ Tutorials Repository: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/ros_intro.md
- Gazebo Sim Fuel Models: https://app.gazebosim.org/fuel/models
- SDF Gazebo Documentation: https://gazebosim.org/docs/latest/sdf_worlds/
- SDF Documentaion: http://sdformat.org/tutorials?tut=spec_materials
