package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

  public static final int indexerMotorID = 0;
  public static final int runIdexerButtonID = 2;

  public static final AngularVelocity indexerAngularVelocity = RadiansPerSecond.of(5);
  public static final double indexerDutyCycleValue = 0.5;

  public static enum IndexerDriveMode {
    Velocity,
    DutyCycle
  }

  public static IndexerDriveMode indexerDriveMode = IndexerDriveMode.DutyCycle;

  public static final class indexerConfigs {
    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
  }
}
