package frc.robot.subsystems.kitCollector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class CollectorConstants {
  public static final double kP = 0.1;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final AngularVelocity defaultCollectorVelocity = RotationsPerSecond.of(5);
  public static final int motorID = 44;
}
