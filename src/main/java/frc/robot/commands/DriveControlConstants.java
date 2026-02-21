package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class DriveControlConstants {
  public static final double yawControlKp = 6.5;
  public static final double yawControlKD = 0.;
  public static final double controllerDeadband = 0.1;
  public static final double yawSetpointTolerance = 0.4;
  public static final Angle largeAngleJumpThreshold = Degrees.of(10);
  public static final double linearFilterTimeConstant = 0.03;
  public static final double linearFilterPeriod = 0.020;
}
