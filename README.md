# autopilots

## Software in the Loop (SITL)
#### To run SITL either cd to ardupilot/ArduCopter and run sim_vehicles.py -w --map --console or you can specify from the home directory like this sim_vehicles.py -v ArduCopter

## Gazebo
### Simulating the virtual environment
#### You can edit how the Gazebo simulated world looks like by either making a script, building an environment, or editing an existing world. The script is written in XML which is like HTML. XML defines back-end descriptions and functions whereas HTML defines UI. The individual worlds and model scripts are written .sdf format. There are other dependencies to also include, such as referenced images and building with pre-existing models.
##### SDF Documentation: http://sdformat.org/spec?ver=1.8&elem=sdf


## IQ Sim Tutorials
In one terminal, run sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
In the second terminal run gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world

As of now, my .bashrc file contains a source for ROS 1 (Noetic). Later, if I want to develop with ROS 2 just be aware to edit the bashrc file and manually source ROS 1 instead (vice versa).
Manually source: source /opt/ros/noetic/setup.bash
