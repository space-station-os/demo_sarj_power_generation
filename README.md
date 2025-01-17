# Space Station Power Generation and SARJ Control Demo

## Command and published output
```
$ cd space_station_os_dev/src/space_station_power_demo
$ colcon build
$ source install/setup.bash
$ ros2 run demo_sarj_power_generation demo_sarj_power_generation
```
Then the ROS node "space_station_power_demo" publishes two messages, generated power [W] and battery level [Wh] (or state of charge, SoC). They can be shown by rqt_plot.
If you plot them, graphs are like this:
![image](https://github.com/user-attachments/assets/6fdb9c3b-9d36-4d80-9cfc-9ae2ba2378f5)

## Simulation Overview
The earth orbit around the Sun and space station orbit and attitude are simulated.
### Dynamics
The earth rotetes around the Sun. Its orbit is perfect circle.
The space station rotetes around the earth. Its orbit is perfect circle. Orbital elements such as altitde, RAAN and inclination can be set. Initial attitude is set as Euler angle Z->Y->X. And angular velocity is set as roll, pitch, yaw.
### Power generation
Shade of the space station by the earth is simulated.
And SAP angle to the sun is calculated.
If the space station is not in shade by the earth, its SAP can generate power.
Its amount depends on the SAP angle to the sun. The angle between the normal vector of the solar cell and the vector in the sun direction is $$\theta$$. The amount of power generated is proportional to $$\cos(\theta)$$, with a maximum value of max_generated_power when $$\theta=0$$ degrees.

## Parameters
Space station parameters:
- ss_altitude: altitude of space station orbit.
- ss_raan: RAAN of space station orbit.
- ss_inclination: inclination of space station orbit.
- ss_init_euler_angle: atitude of space station as Euler angle.
- ss_init_w_vec: angular velocity of space station.

Simulation parameters:
- simu_timestep: timestep of simulation [s]
- speed_rate: rate of simulation

These parameters can be set by like:
```
ros2 run demo_sarj_power_generation demo_sarj_power_generation --ros-args -p simu_timestep:="2.0"
```

## Publish
Three messages are published.
- simulation time [s]
- generated power [W]
- battery level [Wh] (or state of charge, SoC).
You can check them by using rosbag or rqt_plot.

## To-do
- Implementation of SARJ (Solar Alpha Rotary Joint) control. By controling SARJ angle, power generation amount can be optimized.
- Split code. All programs are written in src/demo_sarj_power_generation.cpp now. They should be splitted into different files.
