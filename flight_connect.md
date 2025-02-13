# List of Commands to Connect Flight Computer to Flight Controller 💻
<p> Specifically, this is to set up a USB connection between Jetson Orin Nano and the Cube Pilot Orange+. I am sure the procedure is similar for a Raspberry Pi and Cube Pilot Orange. </p>

<p> Open a terminal with mavproxy.py to view information a/b the drone (heart beat connections and to send manual commands to drone). A high baud rate is needed for data transmission to establish heartbeat connection through DroneKit. --out forwards connection to udp
port to speed up transmission too. Change connection permissions for ttyACM0.</p>

```
sudo chmod 666 /dev/ttyACM0
mavproxy.py --master=dev/ttyACM0 -- baudrate 115200 --out=udp:127.0.0.1:14550
```

<p>Then you'll run as normal the python auto script:</p>

```
python connect_to_cube.py --connect 127.0.0.1:14550
```

<p> Within mavproxy.py, adjust these parameters:</p>

```
param set SR0_EXT_STAT 5
param set SR0_EXTRA1 10
param set SR0_EXTRA2 10
param set SR0_EXTRA3 3
param set SR0_POSITION 10
param set SR0_RAW_CTRL 3
param set SR0_RAW_SENS 3
param set SR0_RC_CHAN 3
```

<p> This works, but our current Cube Pilot Orange runs MAVLink version 3 (a custom library mode), so we get some errors outputted but everything runs normally - the drone will fly. I haven't tried this yet, but we can set it to MAVLink 2 within Mission Planner. <br>
Mission Planner -> Config -> Full Parameter List <br>
and set: </p>

```
SERIAL0_PROTOCOL = 2  (MAVLink2)
SERIAL1_PROTOCOL = 2
```

## Troubleshooting

<p>Check if ACM0 is available, or choose list of available ones</p>

```
ls /dev/ttyACM*
```
