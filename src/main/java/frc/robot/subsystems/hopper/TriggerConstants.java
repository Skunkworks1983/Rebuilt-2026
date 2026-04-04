package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class TriggerConstants {
  public static CANBus triggerCanBus = new CANBus("peripherals_2026");
  public static final double disconnectedDebounceTimeSec = 0.5;

  /** CAN ID for the trigger. */
  public static final int leftTriggerMotorID = 35;

  public static final int rightTriggerMotorID = 34;

  // PID for trigger
  public static final double triggerKP = 2.0;
  public static final double triggerKD = 0.0;

  // Feed forwards for the trigger
  public static final double triggerKS = 1.0;
  public static final double triggerKV = 0.75;
  public static final double triggerKA = 0.0;

  /** Alignment configuration for the trigger motor. */
  public static final InvertedValue primaryMotorInvert = InvertedValue.CounterClockwise_Positive;

  public static final double triggerGearRatio = 19.2; // TODO get actual gear ratio

  /** Stator current limit in Amps for the trigger motor. */
  public static final double triggerStatorCurrentLimitAmps = 100;

  public static final AngularVelocity triggerAngularVelocitySetpoint = RotationsPerSecond.of(25);
  public static final LoggedTunableNumber triggerAngularVelocityRotationsPerSecond =
      new LoggedTunableNumber(
          "Hopper/Trigger/AngularVelocity [Rps]",
          triggerAngularVelocitySetpoint.in(RotationsPerSecond));
  public static final LoggedTunableNumber triggerDutyCycleSpeedFeed =
      new LoggedTunableNumber("Hopper/Trigger/DutyCycleOutFeed", -1);
  public static final LoggedTunableNumber triggerDutyCycleSpeedReverse =
      new LoggedTunableNumber("Hopper/Trigger/DutyCycleOutReverse", .2);

  public static final double triggerSupplyCurrentLimitAmps = 50;

  public static Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);

  public static Voltage sysIdStepSize = Volts.of(2);
}
