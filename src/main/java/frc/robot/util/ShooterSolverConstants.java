package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterSolverConstants {
  public static final Rotation2d hubSafetyAngle = new Rotation2d();
  public static double gravity = 9.80665; // m / s^2
  public static int maxIterations = 5;
  public static double maxError = 0.001;

  public static double weightTheta = 1.0;
  public static double weightVelocity = 1.0;
}
