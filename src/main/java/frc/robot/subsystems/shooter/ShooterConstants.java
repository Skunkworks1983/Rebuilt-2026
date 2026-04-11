package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Constants used by the Shooter subsystem (flywheel and pivot)
 *
 * @author Eddy Walsh
 */
public class ShooterConstants {

  /** CAN Bus name for the shooter motors. */
  public static final CANBus shooterCan = new CANBus("peripherals_2026");

  public static final double disconnectedDebounceTimeSec = 0.5;

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);
  public static final Voltage sysIdStepSize = Volts.of(2);

  // FLYWHEEL CONSTANTS

  /** CAN ID for the primary flywheel motor. */
  public static final int leftFlywheelMotorID = 27;

  /** CAN ID for the secondary flywheel motor. */
  public static final int rightFlywheelMotorID = 21;

  public static final MotorAlignmentValue flywheelMotorAlignment = MotorAlignmentValue.Opposed;

  // PID for flywheel
  public static final double flywheelKP = 12.2;
  public static final double flywheelKD = 0.0;

  // Feed forwards for the flywheel
  public static final double flywheelKS = 0.5;
  public static final double flywheelKV = 0.2;
  public static final double flywheelKA = 10;

  // PID for flywheel
  public static final double lowerFlywheelKP = 9.75;
  public static final double lowerFlywheelKD = 0.0;

  // Feed forwards for the flywheel
  public static final double lowerFlywheelKS = 0.027988;
  public static final double lowerFlywheelKV = 0.075044;
  public static final double lowerFlywheelKA = 3.6181;

  /** Maximum velocity of the flywheel in rotations per second. */
  public static final AngularVelocity flywheelMaxVelocityRotationsPerSec =
      RotationsPerSecond.of(85.6667);

  /** Maximum acceleration of the flywheel in rotations per second squared. */
  public static final AngularAcceleration flywheelMaxAcceleration =
      RotationsPerSecondPerSecond.of(200.0);

  /** Gear ratio of the flywheel mechanism. */
  public static final double flywheelGearRatio = 1.2;

  /** Stator current limit in Amps for the flywheel motors. */
  public static final Current flywheelSupplyCurrentLimitAmps = Amps.of(80);

  /** Invert configuration for the primary flywheel motor. */
  public static final InvertedValue leftMotorInverted = InvertedValue.CounterClockwise_Positive;

  /** Minimum physical pitch angle of the shooter. */
  public static final Angle minShooterPitch = Degrees.of(47.861);

  /** Maximum physical pitch angle of the shooter. */
  public static final Angle maxShooterPitch = Degrees.of(84.193);

  /** Moment of inertia of the flywheel mechanism. */
  public static final MomentOfInertia flywheelInertia = KilogramSquareMeters.of(0.01);

  // This isn't used in the code, but I want to keep it for reference when setting
  // magnet offset
  public static final Angle pivotStartAngle = Degrees.of(84.193);

  /** Initial pose of the shooter on the robot in WPILIb robot coordinate system. */
  public static final Pose3d shooterInitalPosition =
      new Pose3d(
          new Translation3d(Inches.of(-10.93875), Meters.of(0), Inches.of(19.068799)),
          new Rotation3d(Degrees.of(0), pivotStartAngle, Degrees.of(180)));

  /** Radius of the flywheel. */
  public static final Distance flywheelRadius = Inches.of(2);

  /** Simulation PID for the flywheel. */
  public static final Slot0Configs simFlywheelConfigs =
      new Slot0Configs().withKP(70).withKD(0).withKV(0).withKS(0).withKA(0);

  /** Maximum linear velocity of the flywheel surface. */
  public static final LinearVelocity flywheelMaxLinearVelocity =
      MetersPerSecond.of(
          flywheelMaxVelocityRotationsPerSec.in(RadiansPerSecond) * flywheelRadius.in(Meters));

  /** Allowable error tolerance for the flywheel velocity. */
  public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(3);

  // PIVOT CONSTANTS

  /** CAN ID for the primary pivot motor. */
  public static final int pivotMotorId = 29;

  /** Allowable error tolerance for the pivot position. */
  public static final Angle pivotPositionTolerence = Degrees.of(4.5);

  // PID for pivot
  public static final double pivotkP = 160;
  public static final double pivotkD = 0;

  // Feedforwards for pivot
  public static final double pivotkS = 0.35;
  public static final double pivotkV = 0;
  public static final double pivotkA = 0.;

  public static final InvertedValue pivotMotorInverted = InvertedValue.Clockwise_Positive;

  public static final Current pivotStatorCurrentLimit = Amps.of(120);

  public static final double pivotSensorToMechanismRatio = 4.;
  public static final double pivotMotorToSensorRatio = 100.0 * (15.0 / 20.0);

  public static final AngularVelocity defaultUpperFlywheelVelocityGoal =
      RotationsPerSecond.of(46.3);
  public static final AngularVelocity defaultLowerFlywheelVelocityGoal =
      defaultUpperFlywheelVelocityGoal.times(1.15);
  public static final Angle defaultPitchGoal = Degrees.of(64.95);

  public static final AngularVelocity defaultVelocityTransferFar = RotationsPerSecond.of(75);
  public static final AngularVelocity defaultLowerFlywheelVelocityTransfer =
      defaultVelocityTransferFar.times(1.15);
  public static final Angle defaultPitchTransferFar = Degrees.of(50);

  // SENSOR CONSTANTS
  public static final int pivotEncoderId = 28;
  public static final double pivotEncoderAbsoluteOffset = -0.847168;
  public static final double pivotEncoderTotalOffset =
      pivotEncoderAbsoluteOffset + (pivotStartAngle.in(Rotations) * (pivotSensorToMechanismRatio));
  public static final SensorDirectionValue pivotEncoderDirection =
      SensorDirectionValue.Clockwise_Positive;

  public static final int beamBreakSensorChannel = 0;

  public static final int shooterCandiId = 30;

  public static final ReverseLimitSourceValue pivotReverseLimitSource =
      ReverseLimitSourceValue.RemoteCANdiS2;
  public static final ForwardLimitSourceValue pivotForwardLimitSource =
      ForwardLimitSourceValue.RemoteCANdiS1;

  public static final AngularVelocity pivotMaxVelocity = RotationsPerSecond.of(55);

  public static final AngularAcceleration pivotMaxAcceleration =
      RotationsPerSecondPerSecond.of(195);

  public static final Angle pivotTrenchAngle = Degrees.of(79.5);

  public static final Angle defaultPitchTransferClose = Degrees.of(63);

  public static final AngularVelocity defaultVelocityTransferClose = RotationsPerSecond.of(64);

  public enum FlywheelLevel {
    Upper,
    Lower
  }
}
