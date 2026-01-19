package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterConstants {
  public static final CANBus shooterCan = new CANBus("*");

  public static final int flywheelMotorOneId = 0;
  public static final int flywheelSecondaryMotorId = 1;

  public static final double flywheelKP = 0.1;
  public static final double flywheelKD = 0.0;
  public static final double flywheelKS = 0.0;
  public static final double flywheelKV = 0.0;
  public static final double flywheelKA = 0.0;

  public static final double flywheelMaxVelocityRotationsPerSec = 5000.0;
  public static final double flywheelMaxAccelerationRotationsPerSec = 200.0;
  public static final double flywheelMaxJerkRotationsPerSecCubed = 100;

  public static final double flywheelGearRatio = 0.0;

  public static final MotorAlignmentValue flywheelFollowerMotorAlignment =
      MotorAlignmentValue.Opposed;

  public static final double flywheelStatorCurrentLimitAmps = 120.0;

  public static final InvertedValue flywheelMotorInverted = InvertedValue.Clockwise_Positive;

  public static int pivotMotorOneId;

public static int pivotSecondaryMotorId;

public static MotorAlignmentValue pivotFollowerMotorAlignment;
}
