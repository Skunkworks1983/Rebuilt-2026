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
  public static final int leftTriggerMotorID = 26;

  public static final int rightTriggerMotorID = 22;

  // PID for trigger
  public static final double triggerKP = 8.2;
  public static final double triggerKD = 0.0;

  // Feed forwards for the trigger
  public static final double triggerKS = 0.4;
  public static final double triggerKV = 0.;
  public static final double triggerKA = 3.8;

  /** Alignment configuration for the trigger motor. */
  public static final InvertedValue primaryMotorInvert = InvertedValue.CounterClockwise_Positive;

  public static final double triggerGearRatio = 24.0 / 15.0;

  /** Stator current limit in Amps for the trigger motor. */
  public static final double triggerStatorCurrentLimitAmps = 100;

  public static final AngularVelocity triggerAngularVelocitySetpoint = RotationsPerSecond.of(50);
  public static final LoggedTunableNumber triggerAngularVelocityRotationsPerSecond =
      new LoggedTunableNumber(
          "Hopper/Trigger/AngularVelocity [Rps]",
          triggerAngularVelocitySetpoint.in(RotationsPerSecond));

  public static final double triggerSupplyCurrentLimitAmps = 80;

  public static Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);

  public static Voltage sysIdStepSize = Volts.of(2);
}
