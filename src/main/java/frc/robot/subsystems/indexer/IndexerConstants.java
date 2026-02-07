package frc.robot.subsystems.indexer;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {

    public static final int indexerMotorID = 0;
    public static final int runIdexerButtonID = 2;

    public static final AngularVelocity indexerAngularVelocity = Units.RotationsPerSecond.of(2);

    public static final class indexerConfigs {
      public static final double kP = 0.1;
      public static final double kI = 0.01;
      public static final double kD = 0.01;
    
  }
}
