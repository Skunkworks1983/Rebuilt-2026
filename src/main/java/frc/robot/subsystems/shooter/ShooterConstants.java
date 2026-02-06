package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
  public static final CANBus shooterCan = new CANBus("*"); //TODO: Enter canbus

  public static final double disconnectedDebounceTimeSec = 0.5;

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(2);
  public static final Voltage sysIdStepSize = Volts.of(2);

  // FLYWHEEL CONSTANTS

  /** CAN ID for the primary flywheel motor. */
  public static final int flywheelMotorOneId = 0; //TODO: Motor IDS

  /** CAN ID for the secondary flywheel motor. */
  public static final int flywheelSecondaryMotorId = 1;

  // PID for flywheel
  public static final double flywheelKP = 0.1; // TODO: Flywheel Tuning
  public static final double flywheelKD = 0.0;

  // Feed forwards for the flywheel
  public static final double flywheelKS = 0.0;
  public static final double flywheelKV = 0.0;
  public static final double flywheelKA = 0.0;

  /** Maximum velocity of the flywheel in rotations per second. */
  public static final AngularVelocity flywheelMaxVelocityRotationsPerSec =
      RotationsPerSecond.of(85.6667);

  /** Maximum acceleration of the flywheel in rotations per second squared. */
  public static final AngularAcceleration flywheelMaxAcceleration =
      RotationsPerSecondPerSecond.of(200.0);

  /** Gear ratio of the flywheel mechanism. */
  public static final double flywheelGearRatio = 1.;

  /** Alignment configuration for the follower flywheel motor against the primary motor. */
  public static final MotorAlignmentValue flywheelFollowerMotorAlignment =
      MotorAlignmentValue.Opposed;

  /** Stator current limit in Amps for the flywheel motors. */
  public static final Current flywheelStatorCurrentLimitAmps = Amps.of(120);

  /** Invert configuration for the primary flywheel motor. */
  public static final InvertedValue flywheelMotorInverted = InvertedValue.Clockwise_Positive;

  /** Minimum physical pitch angle of the shooter. */
  public static final Angle minShooterPitch = Degrees.of(0); // TODO: Acutal Vals

  /** Maximum physical pitch angle of the shooter. */
  public static final Angle maxShooterPitch = Degrees.of(80);

  /** Moment of inertia of the flywheel mechanism. */
  public static final MomentOfInertia flywheelInertia = KilogramSquareMeters.of(0.01);

  /** Initial pose of the shooter on the robot. */
  public static final Pose3d shooterInitalPosition =
      new Pose3d(
          new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(-40), Degrees.of(0)));

  /** Radius of the flywheel. */
  public static final Distance flywheelRadius = Inches.of(4);

  /** Simulation PID for the flywheel. */
  public static final Slot0Configs simFlywheelConfigs = new Slot0Configs().withKP(30);

  /** Maximum linear velocity of the flywheel surface. */
  public static final LinearVelocity flywheelMaxLinearVelocity =
      MetersPerSecond.of(
          flywheelMaxVelocityRotationsPerSec.in(RadiansPerSecond) * flywheelRadius.in(Meters));

  /** Allowable error tolerance for the flywheel velocity. */
  public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(0.5);
    
  // PIVOT CONSTANTS

  /** CAN ID for the primary pivot motor. */
  public static final int pivotMotorId = 0; // TODO: SET MOTOR IDS

  /** CAN ID for the secondary pivot motor. */
  public static final int pivotSecondaryMotorId = 1;

  /** Allowable error tolerance for the pivot position. */
  public static final Angle pivotPositionTolerence = Degrees.of(3);
    
  // PID for pivot
  public static final double pivotkP = 0; // TODO: SET VALUES
  public static final double pivotkD = 0;

  // Feedforwards for pivot
  public static final double pivotkS = 0;
  public static final double pivotkV = 0;
  public static final double pivotkA = 0;
}
