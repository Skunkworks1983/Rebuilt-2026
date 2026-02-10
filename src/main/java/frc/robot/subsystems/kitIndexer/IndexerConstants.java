package frc.robot.subsystems.kitIndexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
  public static final double kP = 10;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final AngularVelocity defaultIndexerVelocity = RotationsPerSecond.of(5);

  public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
  public static final int motorID = 43;
}
