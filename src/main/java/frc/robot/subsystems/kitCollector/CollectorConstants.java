package frc.robot.subsystems.kitCollector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;

public class CollectorConstants {
  public static final double kP = 6;
  public static final double kD = 0.0;
  public static final double kS = 0.015988;
  public static final double kV = 0.0951044;
  public static final double kA = 0.0;

  public static final AngularVelocity defaultCollectorVelocity = RotationsPerSecond.of(45);
  public static final int motorID = 44;
  public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
  public static final double defaultDutyCycle = 0.5;
}
