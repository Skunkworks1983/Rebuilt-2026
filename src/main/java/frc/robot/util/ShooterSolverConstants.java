package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ShooterSolverConstants {
  public static final Rotation2d hubSafetyAngle = new Rotation2d();
  public static final Distance minShootDistanceHub = Meters.of(0.5);
  public static double gravity = 9.80665; // m / s^2
  public static final Angle defaultPitch = Degrees.of(45);
  public static final double drag = 0.05;
  public static final double maxDragMultiplier = 1.5;
  public static final double flywheelEfficiency = 0.4;

  // CONSTRAINED HEIGHT SOLVER
  public static final Distance hubHeightClearance = Meters.of(1);

  // ITERATIVE SOLVER
  public static int maxIterations = 30;
  public static double maxError = 0.001;
  public static double weightTheta = 1.;
  public static double weightVelocity = 1.;
  public static final double changeRate = 0.85;

  // SHOT LATENCY MODEL
  public static double defaultShotLatencySec = 0.10;
  public static double velocityDragMultiplier = 0.5;
  public static final double minShotLatencySec = 0.015;
  public static final double maxShotLatencySec = 0.200;
  public static final double shotLatencyModelLearningRate = 0.15;
  public static final double lowerFlywheelVelocityMultiplierDefault = 1.1;

  public static final LoggedTunableNumber lowerFlywheelVelocityMultiplier =
      new LoggedTunableNumber(
          "Shooter Solver/LowerFlywheelVelocityMultiplier", lowerFlywheelVelocityMultiplierDefault);
  public static final Angle transferShotSafetyMargin = Degrees.of(4);
}
