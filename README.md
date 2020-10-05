# victor_energy_bmv_700
ROS package to control the battery monitor device.
At this point, the node only can reads the data that the device is broadcasting.

# Dependencies

* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs.git)

# Available params

* port: serial used for the connection

* charge: constant charge value (in Ah)

# Launch the node

```
$ rosrun victor_energy_bmv_700 victor_energy_bmv_700_node.py
```

# Topics

## State

To read the state of the node

```
~state (type robotnik_msgs/State)

```

## BatteryStatus

To read the status of the robot's battery

```
~victor_energy_bmv_700_status (type sensor_msgs/BatteryState)

```

# Services

## To switch on/off the Battery monitoring republisher

```
~toggle (type std_srvs/SetBool)

```


