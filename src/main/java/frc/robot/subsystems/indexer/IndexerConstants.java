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
  public static CANBus indexerCanBus = new CANBus("rio");
  public static final double disconnectedDebounceTimeSec = 0.5;

  /** CAN ID for the indexer. */
  public static final int indexerMotorID = 28;

  public static final int runIndexerButtonId = 4;

  // PID for indexer
  public static final double indexerKP = 3.5; // TODO: Indexer Tuning
  public static final double indexerKD = 0.0;

  // Feed forwards for the indexer
  public static final double indexerKS = 1.0;
  public static final double indexerKV = 0.50;
  public static final double indexerKA = 0.0;

  /** Alignment configuration for the indexer motor. */
  public static final InvertedValue indexerMotorAlignment = InvertedValue.CounterClockwise_Positive;

  /** Stator current limit in Amps for the indexer motor. */
  public static final Current indexerStatorCurrentLimitAmps = Amps.of(120);

  public static final AngularVelocity indexerAngularVelocity = RotationsPerSecond.of(5);

  public static Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);

  public static Voltage sysIdStepSize = Volts.of(2);
}
