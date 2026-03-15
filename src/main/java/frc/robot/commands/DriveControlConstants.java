package frc.robot.commands;

import frc.robot.util.LoggedTunableNumber;

public class DriveControlConstants {
  public static final LoggedTunableNumber YAW_CONTROL_KP =
      new LoggedTunableNumber("Drive Control/Yaw Control KP", 4.25);
  public static final LoggedTunableNumber YAW_CONTROL_KD =
      new LoggedTunableNumber("Drive Control/Yaw Control KD", 0.0);
  public static final LoggedTunableNumber YAW_CONTROL_DEADBAND =
      new LoggedTunableNumber("Drive Control/Yaw Control Deadband", 0.1);
  public static final LoggedTunableNumber YAW_SETPOINT_TOLERANCE_RADIANS =
      new LoggedTunableNumber("Drive Control/Yaw Setpoint Tolerance Radians", 0.08);
  public static final LoggedTunableNumber YAW_MAX_VELOCITY =
      new LoggedTunableNumber("Drive Control/Yaw Max Velocity Radians Per Second", 12.0);
  public static final LoggedTunableNumber YAW_MAX_ACCELERATION =
      new LoggedTunableNumber("Drive Control/Yaw Max Acceleration Radians Per Second^2", 28.0);
}
