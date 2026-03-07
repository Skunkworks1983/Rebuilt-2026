package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Constants used for the physics solver and aiming calculations in {@link ShooterCalculations}.
 *
 * @author Eddy Walsh
 */
public class ShooterSolverConstants {
  /** Safety buffer angle for avoiding the hub when transferring from neutral zone. */
  public static final Rotation2d hubSafetyAngle = new Rotation2d();

  public static double gravity = 9.80665; // m / s^2

  /** Default pitch angle to use when other calculations fail or default is needed. */
  public static final Angle defaultPitch = Degrees.of(70);

  /** Air drag coefficient approximation. */
  public static LoggedTunableNumber drag =
      new LoggedTunableNumber("Shooter/Air Drag Coefficient", 0.085);

  /** Maximum multiplier to apply for drag compensation. */
  public static final double maxDragMultiplier = 3;

  /**
   * Efficiency factor for the flywheel (1 is 100%, 0.1 is 10%). This should at a maximum be 0.5 as
   * half the velocity is converted to spin
   */
  public static final LoggedTunableNumber flywheelEfficiency =
      new LoggedTunableNumber("Flywheel Efficiency", 0.415);

  // CONSTRAINED HEIGHT SOLVER
  /** Vertical clearance required above the hub for specialized constrained shots. */
  public static LoggedTunableNumber hubHeightClearance =
      new LoggedTunableNumber("Shooter/Hub Height Clearance Meters", 0.5);

  public static final LoggedTunableNumber lowerFlywheelVelocityMultiplier =
      new LoggedTunableNumber("Lower Flywheel Velocity Multiplier", 1.15);

  // ITERATIVE SOLVER
  /** Maximum iterations for the numerical solver loop. */
  public static int maxIterations = 30;

  /** Maximum allowable error in the solver's convergence. */
  public static double maxError = 0.001;

  /** Optimization weight for pitch angle changes in the solver. */
  public static double weightTheta = 1.;

  /** Optimization weight for velocity changes in the solver. */
  public static double weightVelocity = 1.;

  /** Learning rate/step size for the gradient descent solver. */
  public static final double changeRate = 0.85;

  // SHOT LATENCY MODEL
  /** Default time delay from code trigger to shot release. */
  public static double defaultShotLatencySec = 0.1;

  /** Minimum reasonable shot latency. */
  public static final double minShotLatencySec = 0.015;

  /** Maximum reasonable shot latency. */
  public static final double maxShotLatencySec = 0.200;

  /** Learning rate for the shot latency correction model. */
  public static final double shotLatencyModelLearningRate = 0.15;

  public static final Angle transferShotSafetyMargin = Degrees.of(10);

  public static final Distance closeShotDistance = Meters.of(1.25);

  public static final LoggedTunableNumber closeShotClearenceMultiplier =
      new LoggedTunableNumber("Shooter/Close Shot Clearance Multiplier", 2.0);

  public static final LoggedTunableNumber closeShootVelocity =
      new LoggedTunableNumber("Shooter/Close Shoot Velocity RPS", 50);
  public static final LoggedTunableNumber closeShootAngle =
      new LoggedTunableNumber("Shooter/Close Shoot Angle Deg", 70);
}
