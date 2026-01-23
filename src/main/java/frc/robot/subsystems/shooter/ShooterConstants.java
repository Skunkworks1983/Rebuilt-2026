package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

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

  public static final double flywheelGearRatio = 1.;

  public static final MotorAlignmentValue flywheelFollowerMotorAlignment =
      MotorAlignmentValue.Opposed;

  public static final double flywheelStatorCurrentLimitAmps = 120.0;

  public static final InvertedValue flywheelMotorInverted = InvertedValue.Clockwise_Positive;

  public static int pivotMotorOneId;

  public static int pivotSecondaryMotorId;

  public static MotorAlignmentValue pivotFollowerMotorAlignment;

  public static final MomentOfInertia flywheelInertia = KilogramSquareMeters.of(0.01);

  public static final Pose3d shooterInitalPosition =
      new Pose3d(
          new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-40), Degrees.of(0)));

  public static final Distance flywheelRadius = Inches.of(3);

  public static final Slot0Configs simFlywheelConfigs =
      new Slot0Configs().withKP(10);
}
