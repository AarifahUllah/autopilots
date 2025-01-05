# :ribbon: Autopilots :ribbon:

# :dizzy: Insights :dizzy:

<p> An explanation of all the components and how they connect. I think a lot of this project in terms of drone programming and gettiing to autonomous flight is actually relatively less coding! But understanding all the pieces and how they connect.
That has taken the most time, and unfortunately, there is limited documentation out there that explains all the pieces well. There's ArduPilot, but to find all that you need at once is challenging. I think I'd recommend reading the code base, going through all
ArduPilot files, and seeing examples of people who have programmed drones to fly and their code. I personally really like DroneDojo's videos. IQ Tutorials are also good. Both together gives you a better sense on what to do since IQ teaches you how to set up the
environment and some basics while DroneDojo gives you real life examples of how to go from simulations in Gazebo to real drone builds and autonomy.</p>

## Copter Flight Modes :sparkles:

<p> You'll also see different flight modes: Guided, Stabilize, Auto, etc. There's actually about ~28 built-in modes. But, you can make your own custom modes. You can switch between modes too. When I first started out, I think it wasn't very obvious to me that
there were even different modes. I thought it just somehow all worked in the end, like the drone flying all by itself. An issue for me was when I followed the IQ Sim Tutorials, I kind of expected the drone to start moving on its own, so I wasn't sure why it wasn't doing 
anything after some time. I learned that Guided mode depends on users typing commands constantly to the terminal with MAVProxy in it, and Auto mode relies on a user providing a script to follow. <br>
  
By the way, the script to follow is essentially just a list of commands you could have also typed in Guided mode.</p>

### Helpful Links
- Flight Modes for Copter: https://ardupilot.org/copter/docs/flight-modes.html#full-list-of-flight-modes
- Change Flight Modes with MAVLink: https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
- Pre-programmed Mission with Auto Mode: https://ardupilot.org/copter/docs/auto-mode.html

## MAVLink, MAVProxy :sparkles:

<p> There's MAVLink. MAVLink is essentially how you pass messages back and forth. It's a way for the user to communicate with the drone, in a way that the drone understands. When you first start up MAVProxy in your terminal, you'll see something like this: </p>

<p> By default, you usually are in mode Stabilize. ArduPilot also says to run these commands:</p>
  
```
mode guided
arm throttle
takeoff 15
```
<p> 15 specifies the target altitude to directly fly up to. You have to run them or else your drone will not take off, but what I found to work is that you need to switch modes and run them, so actually I run these to make my simulations with IQ Sim work:</p>

```
mode stablize
arm throttle
mode guided
arm throttle
takeoff 15
```
### Helpful Links
- MAVLink Commands in Guided Mode: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
  
  <p> This is a bit misleading. These are some example commands you can send from Guided mode, but you can also program these in an Autonomous script flight. For example, you can say to mean fly 10 meters forward of the current position: </p>
  
  ```
  message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3576 10 0 0 0 0 0 0 0 0 0 0
  ```
- MAVLink Commands for Auto mode: https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-land

## MAVROS, pymavlink, DroneKit :sparkles:
<p>DroneKit is super helpful, but I don't know why, it looks like they discontinued their documentation pages for it. The best places to look is their GitHub repository (in this Repo's README.md) and examples from DroneDojo and others. I really liked this tutorial by
  Daniel D. McKinnon who explains that he has no idea about programming, but managed to make a drone work for a fun project he wanted to try in just 2 days! Which shows just how easy DroneKit is to work with / being beginner-friendly. It essentially is an abstraction
  above reading MAVLink messages like the example above. You wouldn't have to remember a string of parameters and numbers. </p>

<p> pymavlink, MAVLink for Python, described to be lightweight and good for message passing between drone and Ground Station, and between drones.</p>

### Helpful Links

  - DroneKit-Python Tutorial by Daniel D. McKinnon: https://www.ddmckinnon.com/2015/12/30/idiots-guide-to-dronekit-python-a-journey-to-whoz-chillin/
  - Daniel's Project Source Code: https://www.ddmckinnon.com/wp-content/uploads/2015/12/WHOZ-CHILLIN.txt
  - MAVLink for Python, pymavlink: https://mavlink.io/en/mavgen_python/
  - MAVLink Standard Definitions: [https://mavlink.io/en/messages/](https://mavlink.io/en/messages/common.html)
  - IQ Tutorial on pymavlink: https://www.youtube.com/watch?v=yyt4VjBRG_Y&list=PLy9nLDKxDN68cwdt5EznyAul6R8mUSNou&index=6

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
