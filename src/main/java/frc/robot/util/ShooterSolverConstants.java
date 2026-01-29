package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class ShooterSolverConstants {
  public static final Rotation2d hubSafetyAngle = new Rotation2d();
  public static final Distance minShootDistanceHub = Meters.of(0.5);
  public static final double changeRate = 0.85;

  public static double gravity = 9.80665; // m / s^2
  public static int maxIterations = 30;
  public static double maxError = 0.001;

  public static double weightTheta = 1.;
  public static double weightVelocity = 1.;
}
