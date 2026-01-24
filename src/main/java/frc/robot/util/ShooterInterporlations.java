package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShooterInterporlations {

  /** Key: Distance from target in meters Value: Shooter angle degrees */
  private final InterpolatingDoubleTreeMap shooterAngleMap;

  /** Key: Distance from target in meters Value: Shooter velocity rotations per second */
  private final InterpolatingDoubleTreeMap shooterVelocityMap;

  public ShooterInterporlations() {
    shooterAngleMap = new InterpolatingDoubleTreeMap();
    shooterAngleMap.put(0., 90.);
    shooterAngleMap.put(1., 80.);
    shooterAngleMap.put(2., 70.);
    shooterAngleMap.put(3., 60.);
    shooterAngleMap.put(4., 50.);
    shooterAngleMap.put(5., 45.);

    shooterVelocityMap = new InterpolatingDoubleTreeMap();
    shooterVelocityMap.put(0., 500.);
    shooterVelocityMap.put(10., 5000.);
  }

  public Angle getPivotAngle(Distance distance) {
    return Degrees.of(shooterAngleMap.get(distance.in(Meters)));
  }

  public AngularVelocity getShooterVelocity(Distance distance) {
    return RotationsPerSecond.of(shooterVelocityMap.get(distance.in(Meters)));
  }
}
