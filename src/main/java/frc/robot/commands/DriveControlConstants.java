package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedTunableNumber;

public class DriveControlConstants {
  public static final LoggedTunableNumber YAW_CONTROL_KP =
      new LoggedTunableNumber("Drive Control/Yaw Control KP", 3.7);
  public static final LoggedTunableNumber YAW_CONTROL_KD =
      new LoggedTunableNumber("Drive Control/Yaw Control KD", 0.5);
  public static final LoggedTunableNumber YAW_CONTROL_DEADBAND =
      new LoggedTunableNumber("Drive Control/Yaw Control Deadband", 0.1);
  public static final LoggedTunableNumber YAW_SETPOINT_TOLERANCE_RADIANS =
      new LoggedTunableNumber("Drive Control/Yaw Setpoint Tolerance Radians", 0.5);
  public static final LoggedTunableNumber YAW_MAX_VELOCITY =
      new LoggedTunableNumber("Drive Control/Yaw Max Velocity Radians Per Second", 7.0);
  public static final LoggedTunableNumber YAW_MAX_ACCELERATION =
      new LoggedTunableNumber("Drive Control/Yaw Max Acceleration Radians Per Second^2", 14.0);

  public static final LoggedTunableNumber linearFilterTimeConstant =
      new LoggedTunableNumber("Auto Aim/Linear Filter Time Constant", 0.05);
  public static final LoggedTunableNumber linearFilterPeriod =
      new LoggedTunableNumber("Auto Aim/Linear Filter Period", 0.025);

  public static final Angle largeAngleJumpThreshold = Degrees.of(10);
}
