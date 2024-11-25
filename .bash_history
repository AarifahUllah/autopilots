echo hi
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git checkout Copter-4.0.4
git submodule update --init --recursive
git config --global url.https://.insteadOf git://
git submodule update --init --recursive
cd ArduCopter
ls
la
sim_vehicle.py -w
cd ..
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install gazebo11 libgazebo11-dev
gazebo
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
nano ~/.bashrc
. ~/.bashrc
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
cd ..
ls
cd ardupilot
cd ArduCopter
sim_vehicle.py -w --map --console
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
sim_vehicle.py -w  --map --console
cd ..
ls
cd ardupilot
cd ArduCopter
sim_vehicle.py -w  --map --console
echo export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0 >> ~/.bashrc
cd ~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws
wstool init ~/catkin_ws/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
cd ~
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gazebo --verbose
~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
ls
cd ardupilot
ls
cd ArduSub
ls
cd ..
cd ArduCopter
ls
sim_vehicles.py -w --map --console
sim_vehicle.py -w --map --console
-la
la
ls
nano ~./bash.rc
nano ~/bash.rc
nano ~/.bashrc
sim_vehicle.py 
sim_vehicle.py -v ArduSub
sim_vehicle.py -v ArduSub -f gazebo-iris --map --console
sim_vehicle.py -v ArduSub -f --map --console
sim_vehicle.py -v ArduSub -f  --console
nano ~/.bashrc
python3 --version
ls
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
nano ~/.bashrc
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
nano ~/.bashrc
cd ~/catkin_ws
catkin build
source ~/.bashrc
roslaunch iq_sim runway.launch
cd
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console 
sim_vehicles.py -w --map --console
cd ardupilot
sim_vehicles.py -w --map --console
cd ArduCopter
sim_vehicles.py -w --map --console
cd ..
ls
cd ardupilot_gazebo
sim_vehicles.py -w --map --console
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
ls
cd logs
ls
cd ..
sim_vehicles.py -w --map --console
cd ardupilot/ArduCopter
sim_vehicles.py -w --map --console
cd ..
ls
cd ardupilot_gazebo
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
cd ardupilot-gazebo
ls
cd ardupilot_gazebo
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
cd ardupilot/ArduCopter
sim_vehicle.py -w --map --console
cd ~
/home/aarifah/ardupilot/build/sitl/bin/arducopter -S -I0 -w --model + --speedup 1 --defaults /home/aarifah/ardupilot/Tools/autotest/default_params/copter.parm
pkill -f arducopter
pkill -f mavproxy.py
cd ardupilot/ArduCopter
sim_vehicle.py -w --map --console
cd ~
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
ls
cd catkin_ws
cd
cd catkin_ws
ls
cd src
ls
roslaunch iq_sim launch.py
roslaunch iq_sim runway.py
cd iq_sim
ls
cd launch
ls
cd ..
roslaunch iq_sim runway.launch
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~
cd 
~/startsitl.sh
~/startsitl.sh
kill -f mavproxy.py
pkill -f arducopter
pkill -f mavproxy.py
~/startsitl.sh
pkill -f arducopter
pkill -f mavproxy.py
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
ls
cd ardupilot_gazebo
ls
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
cd ..
ls
cd catkin_ws
ls
cd src
ls
cd mavlink
ls
cd examples
ls
cd ..
ls
cd mavros
ls
cd mavros
ls
cd launch
ls
cd ..
ls
cd iq_sim
ls
cd models
ls
cd ..
roslaunch iq_sim runway.launch
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
cd ..
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
ls
cd catkin_ws
ls
cd src
ls
cd ..
roslaunch iq_sim runway.launch
~/startsitl.sh
cd catkin_ws/src/iq_sim/scripts
ls
cat startsitl.sh
cd ..
cd
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
gazebo
ls
git init
git add .
git commit -m "set up of work environment"
git config --global user.email "aarifahullah@gwu.edu"
git config --global user.name "Aarifah Ullah"
git commit -m "set up of work environment"
git push
git remote add origin https://github.com/AarifahUllah/autopilots.git
git push -u origin main
git push -u origin master
git checkout main
git checkout -b field
git branch
git pull origin master
git branch
clear
gazebo
ls
cd ardupilot_gazebo
ls
la
cd ..
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world
clean
ls
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
pkill -f arducopter
pkill -f mavproxy.py
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
cd ardupilot_gazebo
cd ..
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world
ssh rdc1-orin@10.8.30.17
roslaunch iq_sim runway.launch
ssh rdc1-orin@10.8.30.17
