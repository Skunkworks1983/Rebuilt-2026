package frc.robot.subsystems.simpleShooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ShooterConstants {
  public static final CANBus shooterCan = new CANBus("rio");

  public static final int upperFlywheelId = 46;
  public static final int backFlywheelId = 45;

  public static final double forwardFlywheelKP = 3.5;
  public static final double forwardFlywheelKD = 0.0;
  public static final double forwardFlywheelKS = 1.0994;
  public static final double forwardFlywheelKV = 0.318791;
  public static final double forwardFlywheelKA = 0.72373;

  public static final double forwardFlywheelMaxVelocityRotationsPerSec = 85.6667;
  public static final double forwardFlywheelMaxAccelerationRotationsPerSec = 200.0;
  public static final double forwardFlywheelMaxJerkRotationsPerSecCubed = 100;

  public static final double backFlywheelKP = 3.5;
  public static final double backFlywheelKD = 0.0;
  public static final double backFlywheelKS = 1.099;
  public static final double backFlywheelKV = 0.318791;
  public static final double backFlywheelKA = 0.72373;

  public static final double backFlywheelMaxVelocityRotationsPerSec = 85.6667;
  public static final double backFlywheelMaxAccelerationRotationsPerSec = 200.0;
  public static final double backFlywheelMaxJerkRotationsPerSecCubed = 100;

  public static final double flywheelGearRatio = 1.;

  public static final double flywheelStatorCurrentLimitAmps = 120.0;

  public static final InvertedValue flywheelMotorInverted = InvertedValue.CounterClockwise_Positive;

  public static final MomentOfInertia flywheelInertia = KilogramSquareMeters.of(0.01);

  public static final Pose3d shooterInitalPosition =
      new Pose3d(
          new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-40), Degrees.of(0)));

  public static final Distance flywheelRadius = Inches.of(4);

  public static final Slot0Configs simFlywheelConfigs = new Slot0Configs().withKP(30);

  public static final LinearVelocity flywheelMaxLinearVelocity =
      MetersPerSecond.of(
          RotationsPerSecond.of(85.66667).in(RadiansPerSecond) * flywheelRadius.in(Meters));

  public static final AngularVelocity defaultForwardFlywheelVelocity = RotationsPerSecond.of(55);
  public static final AngularVelocity defaultBackFlywheelVelocity = RotationsPerSecond.of(55);

  public static Angle shooterPitch = Degrees.of(45);
}
