# Set up CAN 

```
cd src/CANOpenRobotController/script/
./initCAN0.sh

```

# Launch 

```
roslaunch CORC multi_m1_real.launch 
```
# Commands to run in rqt during calibration:
In m1_x
1) controller mode set to zero calibration to set encoder
2) put in zero torque 
3) press set_rom and move ankle to calculate ROM
3) unpress set_rom and switch controller mode to center_angle 
4) now M1 controller in zero velocity so press set_mvc and push and pull with ankle to calculate mvc
5) unpress set_mvc

# Controllers:

1) transparency is transparent control for zero torque
2) center angle is isometric, zero velocity in the center angle of ROM (if you want to change angle use pos_isometric)
3) isokinetic is moving with constants velocity in the ROM (parameters can be changed with vel_isokinetic [deg/s], amplitude_isokinetic [deg] and pos_isokinetic = center angle of the isokinetic [deg])

# Reference trajectory

In multi_robot_interaction (rqt) are present some reference trajectory which are published as rostopic in /m1_x/joint_commands to activate them:

- interaction_torque = isometric_force
- trajectory_mode can be:
1) flat
2) sine
3) trapezoidal
use rqt to change parameters of references
