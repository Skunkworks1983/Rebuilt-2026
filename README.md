# The code for the 2026 FRC Season: Rebuilt
This code features IO abstracted subsystems, simulation support for most mechanisms, full AdvantageKit logging, and multithreaded drive odometry.

## Controls
### Driver Controllers
#### Port 0 Joystick
Stick - Translate

#### Port 1 Joystick
Stick - Rotate

Trigger - Shoot Auto Aimed (Automatically indexes when ready)

### Operator Controller (Xbox)
Right Trigger - Shoot Hub, No Yaw Control, Odometry based shooter speed and hood angle

Left Trigger - Shoot Transfer, Yaw Control, Odometry based shooter speed and hood angle

Right Bumper - Shoot Hub, Preset Setpoints (No odometry)

Left Bumper - Shoot Transfer, Preset Setpoints (No odometry)

Y - Collector Up

B - Reverse Hopper and Collector

X - Force Index into Shooter

A - Intake / Put Collector Down

## Setting Hood Absolute Position
1. Use a wrench to push hood all the way against lower stop (as in if a ball shot out it would go straight vertically up)
2. In Phoenix Tuner, refresh the absolute encoder and copy the absolute position
3. On line 168 of `ShooterConstants.java` paste the absolution position and put a negative sign in front of it
4. The code will do all gear ratio calculations, just run a deploy
5. As of 2026, there was a bug in the encoder firmware that it would not properly update to use the new offset. Verify in AdvantageScope that when moving the hood the angle moves within the range of 47-84deg. If it does not, re-image the encoder, do a deploy, and it should work.

## Dashboard Layouts
A premade dashboard that displays all subsystem data can be imported into AScope can be found in `/layouts/Advantage Scope Real Data Analysis Layout.json`. Layouts also exist for simulation and swerve module calibration.
