package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShooterInterporlations {

  /**
   * Interpolation for shooting at goal (meaning landing z != 0) Key: Distance from target in meters
   * Value: Shooter angle degrees
   */
  private final InterpolatingDoubleTreeMap shooterGoalAngleMap;

  /**
   * Interpolation for shooting at goal (meaning landing z != 0) Key: Distance from target in meters
   * Value: Shooter velocity rotations per second
   */
  private final InterpolatingDoubleTreeMap shooterGoalVelocityMap;

  /**
   * Interpolation for shooting at goal (meaning landing z == 0) Key: Distance from target in meters
   * Value: Shooter angle degrees
   */
  private final InterpolatingDoubleTreeMap shooterGroundAngleMap;

  /**
   * Interpolation for shooting at goal (meaning landing z == 0) Key: Distance from target in meters
   * Value: Shooter velocity rotations per second
   */
  private final InterpolatingDoubleTreeMap shooterGroundVelocityMap;

  public ShooterInterporlations() {
    shooterGoalAngleMap = new InterpolatingDoubleTreeMap();
    shooterGoalAngleMap.put(0., 90.);
    shooterGoalAngleMap.put(1., 80.);
    shooterGoalAngleMap.put(2., 70.);
    shooterGoalAngleMap.put(3., 60.);
    shooterGoalAngleMap.put(4., 50.);
    shooterGoalAngleMap.put(5., 45.);

    shooterGoalVelocityMap = new InterpolatingDoubleTreeMap();
    shooterGoalVelocityMap.put(0., 500.);
    shooterGoalVelocityMap.put(10., 5000.);

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

  public Angle getPivotAngleGoal(Distance distance) {
    return Degrees.of(shooterGoalAngleMap.get(distance.in(Meters)));
  }

  public AngularVelocity getShooterVelocityGoal(Distance distance) {
    return RotationsPerSecond.of(shooterGoalVelocityMap.get(distance.in(Meters)));
  }

  public Angle getPivotAngleTransfer(Distance distance) {
    return Degrees.of(shooterGroundAngleMap.get(distance.in(Meters)));
  }

  public AngularVelocity getShooterVelocityTransfer(Distance distance) {
    return RotationsPerSecond.of(shooterGroundVelocityMap.get(distance.in(Meters)));
  }
}
