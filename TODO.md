# 🪷 THINGS TO WORK ON  🪷
<P>A list of things that need to be done for navigation simulations and later physical drone implementation.</P>

Both drones in the same world
-[ ] Figure out port connections for SITL, Gazebo, and MAVProxy instances
-[ ] Start two drones with DroneKit using .connect() in same script
-[ ] Connect both drones together in DroneKit and pass messages

Visual streaming and processing
-[ ] MAVLink video streaming, not exactly necessary to see what drone sees but at least we need to connect current visual processing scripts and have navigation with it. e.g. precision landing on ArUco
-[ ] Research and figuring out how to base navigation movements with the seeing the drone. 
-[ ] Switching from searching pattern to getting closer to drone. We are already able to do this with speed_test.py

<p>Before I forget. Actually what could work is that as soon as we spot the drone, we do a distance calculation on it. Then navigation's goal is to move constantly in the direction that
decreases that distace and potential angle. Until it is all 0.</p>

Search patterns
-[ ] Circular / Elliptical search
-[ ] Lissajous search
-[ ] Mothership idea
-[ ] Bounded box search

Mapping and Grids
-[ ] GPS and RTK module research, find something that works both indoors and outdoors.
-[ ] Triangulate distances. We have two starting points, then later a third point with the marker. 
-[ ] Make a map of the field in 1 m increments with corresponding GPS coordinate. May not be needed.
-[ ] Calibrate GPS and RTK modules

Prep for physical world
-[ ] Study how to go from simulation to real world, do we need to download something, change code etc.? Connect Jetson with PixHawk?
-[ ] Figure out how camera is being places on package drone cause I'm assuming it's facing downward but where is the package going to be?

🦋 💞 🌵 🌷 🌻 🐅
