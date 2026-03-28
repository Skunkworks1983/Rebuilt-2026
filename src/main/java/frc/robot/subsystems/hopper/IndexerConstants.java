package frc.robot.subsystems.hopper;

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
import frc.robot.util.LoggedTunableNumber;

public class IndexerConstants {
  public static CANBus indexerCanBus = new CANBus("peripherals_2026");
  public static final double disconnectedDebounceTimeSec = 0.5;

  /** CAN ID for the indexer. */
  public static final int leftIndexerMotorID = 40;

  public static final int rightIndexerMotorID = 0; // TODO get the real ID

  // PID for indexer
  public static final double indexerKP = 2.0; // TODO: Indexer Tuning
  public static final double indexerKD = 0.0;

  // Feed forwards for the indexer
  public static final double indexerKS = 1.0;
  public static final double indexerKV = 0.75;
  public static final double indexerKA = 0.0;

  /** Alignment configuration for the indexer motor. */
  public static final InvertedValue indexerMotorAlignment = InvertedValue.CounterClockwise_Positive;

  public static final double indexerSensorToMechanismRatio = 19.2;

  /** Stator current limit in Amps for the indexer motor. */
  public static final Current indexerStatorCurrentLimitAmps = Amps.of(120);

  public static final AngularVelocity indexerAngularVelocitySetpoint = RotationsPerSecond.of(25);
  public static final LoggedTunableNumber indexerAngularVelocityRotationsPerSecond =
      new LoggedTunableNumber(
          "Indexer/AngularVelocity [Rps]", indexerAngularVelocitySetpoint.in(RotationsPerSecond));
  public static final LoggedTunableNumber indexerDutyCycleSpeedFeed =
      new LoggedTunableNumber("Indexer/DutyCycleOutFeed", 1);
  public static final LoggedTunableNumber indexerDutyCycleSpeedReverse =
      new LoggedTunableNumber("Indexer/DutyCycleOutReverse", -.2);

  public static final LoggedTunableNumber autoUnjamNoBeamBreakTimeoutSeconds =
      new LoggedTunableNumber("Indexer/AutoUnjam/NoBeamBreakTimeout [sec]", 1.5);
  public static final LoggedTunableNumber autoUnjamReverseDurationSeconds =
      new LoggedTunableNumber("Indexer/AutoUnjam/ReverseDuration [sec]", .5);

  public static Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);

  public static Voltage sysIdStepSize = Volts.of(2);
}
