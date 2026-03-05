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
  public static final int upperFlywheelMotorID = 26; // TODO: Motor IDS

  /** CAN ID for the secondary flywheel motor. */
  public static final int lowerFlywheelMotorID = 25;

  // PID for flywheel
  public static final double upperFlywheelKP = 8.0;
  public static final double upperFlywheelKD = 0.0;

  // Feed forwards for the flywheel
  public static final double upperFlywheelKS = 0.027988;
  public static final double upperFlywheelKV = 0.071044;
  public static final double upperFlywheelKA = 3.60181;

  // PID for flywheel
  public static final double lowerFlywheelKP = 9.5;
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
  public static final double flywheelGearRatio = 1.;

  /** Stator current limit in Amps for the flywheel motors. */
  public static final Current flywheelStatorCurrentLimitAmps = Amps.of(120);

  /** Invert configuration for the primary flywheel motor. */
  public static final InvertedValue flywheelMotorInverted = InvertedValue.CounterClockwise_Positive;

  /** Minimum physical pitch angle of the shooter. */
  public static final Angle minShooterPitch = Degrees.of(0); // TODO: Acutal Vals

  /** Maximum physical pitch angle of the shooter. */
  public static final Angle maxShooterPitch = Degrees.of(80);

  /** Moment of inertia of the flywheel mechanism. */
  public static final MomentOfInertia flywheelInertia = KilogramSquareMeters.of(0.01);

  // This isn't used in the code, but I want to keep it for reference when setting
  // magnet offset
  public static final Angle pivotStartAngle = Degrees.of(83);

  /** Initial pose of the shooter on the robot in WPILIb robot coordinate system. */
  public static final Pose3d shooterInitalPosition =
      new Pose3d(
          new Translation3d(Inches.of(-4.483162), Meters.of(0), Inches.of(21.311)),
          new Rotation3d(Degrees.of(0), pivotStartAngle, Degrees.of(0)));

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
  public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(0.5);

  // PIVOT CONSTANTS

  /** CAN ID for the primary pivot motor. */
  public static final int pivotMotorId = 27;

  /** Allowable error tolerance for the pivot position. */
  public static final Angle pivotPositionTolerence = Degrees.of(3);

  // PID for pivot
  public static final double pivotkP = 90; // TODO: SET VALUES
  public static final double pivotkD = 0;

  // Feedforwards for pivot
  public static final double pivotkS = 1;
  public static final double pivotkV = 0;
  public static final double pivotkA = 0;

  public static final InvertedValue pivotMotorInverted = InvertedValue.Clockwise_Positive;

  public static final Current pivotStatorCurrentLimit = Amps.of(120);

  public static final double pivotSensorToMechanismRatio = 4.;
  public static final double pivotMotorToSensorRatio = 100;

  public static final AngularVelocity defaultUpperFlywheelVelocityGoal = RotationsPerSecond.of(50);
  public static final AngularVelocity defaultLowerFlywheelVelocityGoal =
      defaultUpperFlywheelVelocityGoal.times(1.15);
  public static final Angle defaultPitchGoal = Degrees.of(70);

  public static final AngularVelocity defaultVelocityTransfer = RotationsPerSecond.of(60);
  public static final AngularVelocity defaultLowerFlywheelVelocityTransfer =
      defaultVelocityTransfer.times(1.15);
  public static final Angle defaultPitchTransfer = Degrees.of(50);

  // SENSOR CONSTANTS
  public static final int pivotEncoderId = 43;
  public static final double pivotEncoderAbsoluteOffset = -0.582275;
  public static final double pivotEncoderTotalOffset =
      pivotEncoderAbsoluteOffset + (pivotStartAngle.in(Rotations) * (pivotSensorToMechanismRatio));
  public static final SensorDirectionValue pivotEncoderDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  public static final int beamBreakSensorChannel = 0;

  public static final int shooterCandiId = 7;

  public static final ReverseLimitSourceValue pivotReverseLimitSource =
      ReverseLimitSourceValue.RemoteCANdiS1;
  public static final ForwardLimitSourceValue pivotForwardLimitSource =
      ForwardLimitSourceValue.RemoteCANdiS2;

  public static final AngularVelocity pivotMaxVelocity = RotationsPerSecond.of(5);

  public static final AngularAcceleration pivotMaxAcceleration = RotationsPerSecondPerSecond.of(10);
}
