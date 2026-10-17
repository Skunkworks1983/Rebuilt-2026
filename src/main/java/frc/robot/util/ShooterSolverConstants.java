package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Constants used for the physics solver and aiming calculations in {@link ShooterCalculations}.
 *
 * @author Eddy Walsh
 */
public class ShooterSolverConstants {
  public static double gravity = 9.80665; // m / s^2

  /** Air drag coefficient approximation. */
  public static LoggedTunableNumber drag =
      new LoggedTunableNumber("Shooter/Air Drag Coefficient", 0.091); // Original value: 0.084

  /** Maximum multiplier to apply for drag compensation. */
  public static final double maxDragMultiplier = 3;

  /**
   * Efficiency factor for the flywheel (1 is 100%, 0.1 is 10%). This should at a maximum be 0.5 as
   * half the velocity is converted to spin
   */
  public static final LoggedTunableNumber flywheelEfficiency =
      new LoggedTunableNumber("Flywheel Efficiency", 0.495);

  // CONSTRAINED HEIGHT SOLVER
  /** Vertical clearance required above the hub for specialized constrained shots. */
  public static LoggedTunableNumber closeShotApexHeight =
      new LoggedTunableNumber("Shooter/Close Shot Apex Height Meters", 0.5);

  public static LoggedTunableNumber farShotApexHeight =
      new LoggedTunableNumber("Shooter/Far Shot Apex Height Meters", 1.7);
  public static LoggedTunableNumber farShotDistance =
      new LoggedTunableNumber("Shooter/Far Shot Distance Meters", 2.8);

  public static final LoggedTunableNumber lowerFlywheelVelocityMultiplier =
      new LoggedTunableNumber("Lower Flywheel Velocity Multiplier", 1.15);

  // SHOT LATENCY MODEL
  /** Default time delay from code trigger to shot release. */
  public static double defaultShotLatencySec = 0.125;

  /** Minimum reasonable shot latency. */
  public static final double minShotLatencySec = 0.015;

  /** Maximum reasonable shot latency. */
  public static final double maxShotLatencySec = 0.200;

  /** Learning rate for the shot latency correction model. */
  public static final double shotLatencyModelLearningRate = 0.15;

  public static final Angle transferShotSafetyMargin = Degrees.of(20);

  public static final Distance closeShotDistance = Meters.of(1.3);

  public static final LoggedTunableNumber closeShotClearenceMultiplier =
      new LoggedTunableNumber("Shooter/Close Shot Clearance Multiplier", 2.0);

  public static final LoggedTunableNumber closeShootVelocity =
      new LoggedTunableNumber("Shooter/Close Shoot Velocity RPS", 47);
  public static final LoggedTunableNumber closeShootAngle =
      new LoggedTunableNumber("Shooter/Close Shoot Angle Deg", 76.5);

  public static final LoggedTunableNumber shooterDistanceAdder =
      new LoggedTunableNumber("Shooter/Distance Adder", 0.5); //.07

  public static final LoggedTunableNumber mediumShotDistance =
      new LoggedTunableNumber("Shooter/Medium Shot Distance", 2);

  public static final LoggedTunableNumber mediumShotApexHeight =
      new LoggedTunableNumber("Shooter/Medium Shot Apex Height Meters", .85);
}
