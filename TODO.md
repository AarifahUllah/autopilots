# THINGS TO WORK ON

<p> A list of things that need to be done for navigation simulations and later physical drone implementation.</p>

### Both drones in the same world

<p> 
- Figure out port connections for SITL, Gazebo, and MAVProxy instances <br>
- Start two drones with DroneKit using .connect() in same script <br>
- Connect both drones together in DroneKit and pass messages <br> 
</p>

### Visual streaming and processing

<p>
- MAVLink video streaming, not exactly necessary to see what drone sees but at least we need to connect current visual processing scripts and have navigation with it. e.g. precision landing on ArUco <br>
- Research and figuring out how to base navigation movements with the seeing the drone. <br>
- Switching from searching pattern to getting closer to drone. We are already able to do this with speed_test.py <br>

Before I forget. Actually what could work is that as soon as we spot the drone, we do a distance calculation on it. Then navigation's goal is to move constantly in the direction that decreases that distace and potential angle. Until it is all 0.
</p>

### Search patterns

<p>
- Lissajous search <br>
- Mothership idea <br>
- Bounded box search <br>
</p>

### Mapping and Grids

<p>
- GPS and RTK module research, find something that works both indoors and outdoors. <br>
- Triangulate distances. We have two starting points, then later a third point with the marker. <br>
- Make a map of the field in 1 m increments with corresponding GPS coordinate. May not be needed. <br>
- GPS sensor filtering <br>
- Calibrate GPS and RTK modules <br>
</p>

### Prep for physical world

<p>
- Study how to go from simulation to real world, do we need to download something, change code etc.? Connect Jetson with PixHawk? <br>
- Figure out how camera is being places on package drone cause I'm assuming it's facing downward but where is the package going to be? <br>
</p>

🦋 💞 🌵 🌷 🌻 🐅
