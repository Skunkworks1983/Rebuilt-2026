package frc.robot.subsystems.simpleShooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.PivotIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterCalculations.ShooterControls;
import frc.robot.util.ShooterInterporlations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum ShooterState {
    OFF,
    SHOOT_GOAL,
    SHOOT_ALLIANCE_ZONE
  }

  private ShooterState shooterState = ShooterState.OFF;

  private final FlywheelIO flywheel;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0 + .1e-6);
  private Angle pivotSetpoint = Radians.of(0 + .1e-6);

  private final SysIdRoutine sysId;

  private static final LoggedTunableNumber flywheelKP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", ShooterConstants.flywheelKP);
  private static final LoggedTunableNumber flywheelKD =
      new LoggedTunableNumber("Shooter/Flywheel/kD", ShooterConstants.flywheelKD);
  private static final LoggedTunableNumber flywheelKS =
      new LoggedTunableNumber("Shooter/Flywheel/kS", ShooterConstants.flywheelKS);
  private static final LoggedTunableNumber flywheelKV =
      new LoggedTunableNumber("Shooter/Flywheel/kV", ShooterConstants.flywheelKV);
  private static final LoggedTunableNumber flywheelKA =
      new LoggedTunableNumber("Shooter/Flywheel/kA", ShooterConstants.flywheelKA);

  private static final LoggedTunableNumber flywheelMaxAccelerationRotationsPerSec2 =
      new LoggedTunableNumber(
          "Shooter/Flywheel/MaxAccelerationRotationsPerSec2",
          ShooterConstants.flywheelMaxAccelerationRotationsPerSec);
  private static final LoggedTunableNumber flywheelMaxJerkRotationsPerSec3 =
      new LoggedTunableNumber(
          "Shooter/Flywheel/MaxJerkRotationsPerSec3",
          ShooterConstants.flywheelMaxJerkRotationsPerSecCubed);

  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("Shooter/Pivot/kP", 2000);
  private static final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Shooter/Pivot/kD", 0);
  private static final LoggedTunableNumber pivotKS = new LoggedTunableNumber("Shooter/Pivot/kS", 0);
  private static final LoggedTunableNumber pivotKV = new LoggedTunableNumber("Shooter/Pivot/kV", 0);
  private static final LoggedTunableNumber pivotKA = new LoggedTunableNumber("Shooter/Pivot/kA", 0);

  private static final LoggedTunableNumber pivotMaxVelocityRotationsPerSec =
      new LoggedTunableNumber("Shooter/Pivot/MaxVelocityRotationsPerSecond", 2.0);
  private static final LoggedTunableNumber pivotMaxAccelerationRotationsPerSec2 =
      new LoggedTunableNumber("Shooter/Pivot/MaxAccelerationRotationsSec2", 4.0);
  private static final LoggedTunableNumber pivotMaxJerkRotationsPerSec3 =
      new LoggedTunableNumber("Shooter/Pivot/MaxJerkRotationsPerSec3", 4.0);

  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel disconnected", AlertType.kError);
  private final Alert pivotDisconnectedAlert = new Alert("Pivot disconnected", AlertType.kError);

  public Shooter(FlywheelIO flywheel) {
    this.flywheel = flywheel;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(2),
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(flywheelInputs);

    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Pivot", pivotInputs);

    flywheelDisconnectedAlert.set(
        (!flywheelInputs.primaryConnected || !flywheelInputs.followerConnected)
            && Constants.currentMode != Constants.Mode.SIM);

    if (flywheelKP.hasChanged(hashCode())
        || flywheelKD.hasChanged(hashCode())
        || flywheelKS.hasChanged(hashCode())
        || flywheelKV.hasChanged(hashCode())
        || flywheelKA.hasChanged(hashCode())) {
      flywheel.setPID(
          flywheelKP.get(),
          0.0,
          flywheelKD.get(),
          flywheelKS.get(),
          flywheelKV.get(),
          flywheelKA.get());
    }

    if (flywheelMaxAccelerationRotationsPerSec2.hasChanged(hashCode())
        || flywheelMaxJerkRotationsPerSec3.hasChanged(hashCode())) {
      flywheel.setMotionConstraints(
          RotationsPerSecondPerSecond.of(flywheelMaxAccelerationRotationsPerSec2.get()),
          flywheelMaxJerkRotationsPerSec3.get());
    }
  }

  public LinearVelocity getShooterLinearVelocity() {
    // return MetersPerSecond.of(
    // flywheelInputs.primaryVelocity.in(RadiansPerSecond)
    // * ShooterConstants.flywheelRadius.in(Meters));
    return MetersPerSecond.of(
        flywheelSetpoint.in(RadiansPerSecond) * ShooterConstants.flywheelRadius.in(Meters));
  }

  private void runCharacterization(double output) {
    flywheel.setOpenLoopOutput(output);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public void setManualVelocity(AngularVelocity velocity) {
    flywheel.setVelocity(velocity);
  }

  @AutoLogOutput(key = "Shooter/canShoot")
  public boolean canShoot() {
    return true;
  }

  public Angle getPivotAngle() {
    // return pivotInputs.pivotPrimaryPosition;
    return pivotSetpoint;
  }

  public AngularVelocity getFlywheelVelocity() {
    // return flywheelInputs.primaryVelocity;

    return flywheelSetpoint;
  }

  public void setShooterControls(ShooterControls controls) {
    flywheelSetpoint = controls.shooterVelocity();
    pivotSetpoint = controls.shooterPitch();

    Logger.recordOutput(
        "Shooter/Velocity Setpoint Rotations Per Sec", flywheelSetpoint.in(RotationsPerSecond));

    flywheel.setVelocity(flywheelSetpoint);
  }
}
