package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class IndexerConstants {

  public static final int indexerMotorID = 28;
  public static final int runIndexerButtonID = 2;

  public static final AngularVelocity indexerAngularVelocity = RadiansPerSecond.of(5);
  public static final double indexerDutyCycleValue = 0.5;

  public static enum IndexerDriveMode {
    Velocity,
    DutyCycle
  }

  // defines whether the indexer operated by velocity or duty cycle
  public static IndexerDriveMode indexerDriveMode = IndexerDriveMode.DutyCycle;

  public static final class indexerConfigs {
    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
  }
}
