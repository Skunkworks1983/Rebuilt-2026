package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * Helper class for handling interpolating lookups for shooter pitch and velocity based on distance.
 * Manages separate maps for "Goal" shots and "Ground/Transfer" shots.
 *
 * <p>These are not actually used
 *
 * @author Eddy Walsh
 */
public class ShooterInterporlations {

  private final InterpolatingDoubleTreeMap shooterGoalAngleMap;

  /**
   * Interpolation map for shooting at the high goal. Key: Distance from target in meters. Value:
   * Shooter flywheel velocity in rotations per second (stored as double).
   */
  public static final InterpolatingDoubleTreeMap shooterGoalVelocityMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap shooterGroundAngleMap;

  private final InterpolatingDoubleTreeMap shooterGroundVelocityMap;

  /** Initializes the interpolation maps with predefined data points. */
  public ShooterInterporlations() {
    shooterGoalAngleMap = new InterpolatingDoubleTreeMap();
    shooterGoalAngleMap.put(0., 90.);
    shooterGoalAngleMap.put(1., 80.);
    shooterGoalAngleMap.put(2., 70.);
    shooterGoalAngleMap.put(3., 60.);
    shooterGoalAngleMap.put(4., 50.);
    shooterGoalAngleMap.put(5., 45.);

    shooterGroundAngleMap = new InterpolatingDoubleTreeMap();
    shooterGroundAngleMap.put(0., 90.);
    shooterGroundAngleMap.put(1., 80.);
    shooterGroundAngleMap.put(2., 70.);
    shooterGroundAngleMap.put(3., 60.);
    shooterGroundAngleMap.put(4., 50.);
    shooterGroundAngleMap.put(5., 45.);

    shooterGroundVelocityMap = new InterpolatingDoubleTreeMap();
    shooterGroundVelocityMap.put(0., 500.);
    shooterGroundVelocityMap.put(10., 5000.);
  }

  /**
   * Gets the pivot angle for a goal shot at the given distance.
   *
   * @param distance Distance to the goal.
   * @return The interpolated Angle.
   */
  public Angle getPivotAngleGoal(Distance distance) {
    return Degrees.of(shooterGoalAngleMap.get(distance.in(Meters)));
  }

  /**
   * Gets the flywheel velocity for a goal shot at the given distance.
   *
   * @param distance Distance to the goal.
   * @return The interpolated AngularVelocity.
   */
  public AngularVelocity getShooterVelocityGoal(Distance distance) {
    return RotationsPerSecond.of(shooterGoalVelocityMap.get(distance.in(Meters)));
  }

  /**
   * Gets the pivot angle for a transfer/ground shot at the given distance.
   *
   * @param distance Distance to the target.
   * @return The interpolated Angle.
   */
  public Angle getPivotAngleTransfer(Distance distance) {
    return Degrees.of(shooterGroundAngleMap.get(distance.in(Meters)));
  }

  /**
   * Gets the flywheel velocity for a transfer/ground shot at the given distance.
   *
   * @param distance Distance to the target.
   * @return The interpolated AngularVelocity.
   */
  public AngularVelocity getShooterVelocityTransfer(Distance distance) {
    return RotationsPerSecond.of(shooterGroundVelocityMap.get(distance.in(Meters)));
  }

  static {
    shooterGoalVelocityMap.put(1.5, 7.);

    shooterGoalVelocityMap.put(2.5, 8.);
    shooterGoalVelocityMap.put(3.5, 8.5);

    shooterGoalVelocityMap.put(4.5, 12.5);
    shooterGoalVelocityMap.put(5.5, 14.5);

    shooterGoalVelocityMap.put(7.0, 17.0);
  }
}
