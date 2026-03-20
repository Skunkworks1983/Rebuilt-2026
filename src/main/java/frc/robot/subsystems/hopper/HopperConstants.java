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

public class HopperConstants {
  public static CANBus hopperCanBus = new CANBus("peripherals_2026");
  public static final double disconnectedDebounceTimeSec = 0.5;

  /** CAN ID for the hopper. */
  public static final int hopperMotorID = 28;

  // PID for hopper
  public static final double hopperKP = 0.5; // TODO: hopper Tuning
  public static final double hopperKD = 0.0;

  // Feed forwards for the hopper
  public static final double hopperKS = 1.0;
  public static final double hopperKV = 0.75;
  public static final double hopperKA = 0.0;

  /** Alignment configuration for the hopper motor. */
  public static final InvertedValue hopperMotorAlignment = InvertedValue.Clockwise_Positive;

  /** Stator current limit in Amps for the hopper motor. */
  public static final Current hopperStatorCurrentLimitAmps = Amps.of(80);

  public static final AngularVelocity hopperAngularVelocitySetpoint = RotationsPerSecond.of(10);
  public static final LoggedTunableNumber hopperAngularVelocityRotationsPerSecond =
      new LoggedTunableNumber(
          "hopper/AngularVelocity [Rps]", hopperAngularVelocitySetpoint.in(RotationsPerSecond));

  public static final AngularVelocity hopperReverseAngularVelocitySetpoint =
      RotationsPerSecond.of(1);
  public static final LoggedTunableNumber hopperReverseAngularVelocityRotationsPerSecond =
      new LoggedTunableNumber("hopper/ReverseAngularVelocity [Rps]", -10);
  public static final LoggedTunableNumber hopperDutyCycleSpeedFeed =
        new LoggedTunableNumber("hopper/DutyCycleOutFeed", 1);
  public static final LoggedTunableNumber hopperDutyCycleSpeedReverse =
        new LoggedTunableNumber("hopper/DutyCycleOutReverse", -1);

  public static Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);

  public static Voltage sysIdStepSize = Volts.of(2);
}
