package frc.robot.subsystems.shooter;

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
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem responsible for the shooter mechanism, including the flywheel and pivot. Handles
 * pivoting, spinning up the flywheel, and sysid characterization the mechanism.
 *
 * @author Eddy Walsh
 */
public class Shooter extends SubsystemBase {
  private final FlywheelIO forwardFlywheel;
  private final FlywheelIO backFlywheel;

  private final PivotIO pivot;
  private final FlywheelIOInputsAutoLogged forwardFlywheelInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged backFlywheelInputs = new FlywheelIOInputsAutoLogged();
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

  private final Alert forwardFlywheelDisconnectedAlert =
      new Alert("Forward Flywheel disconnected", AlertType.kError);
  private final Alert backFlywheelDisconnectedAlert =
      new Alert("Back Flywheel disconnected", AlertType.kError);
  private final Alert pivotDisconnectedAlert = new Alert("Pivot disconnected", AlertType.kError);

  /**
   * Constructs a new Shooter instance.
   *
   * @param forwardFlywheel The IO implementation for the forward (launch) flywheel.
   * @param forwardFlywheel The IO implementation for the back flywheel.
   * @param pivot The IO implementation for the pivot.
   */
  public Shooter(FlywheelIO forwardFlywheel, FlywheelIO backFlywheel, PivotIO pivot) {
    this.forwardFlywheel = forwardFlywheel;
    this.backFlywheel = backFlywheel;
    this.pivot = pivot;

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
    forwardFlywheel.updateInputs(forwardFlywheelInputs);
    backFlywheel.updateInputs(backFlywheelInputs);
    pivot.updateInputs(pivotInputs);

    Logger.processInputs("Shooter/Flywheel/Forward Flywheel", forwardFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Back Flywheel", backFlywheelInputs);
    Logger.processInputs("Shooter/Pivot", pivotInputs);

    forwardFlywheelDisconnectedAlert.set(!forwardFlywheelInputs.connected);

    backFlywheelDisconnectedAlert.set(!backFlywheelInputs.connected);

    pivotDisconnectedAlert.set(
        (!pivotInputs.primaryConnected || !pivotInputs.followerConnected)
            && Constants.currentMode != Constants.Mode.SIM);

    if (flywheelKP.hasChanged(hashCode())
        || flywheelKD.hasChanged(hashCode())
        || flywheelKS.hasChanged(hashCode())
        || flywheelKV.hasChanged(hashCode())
        || flywheelKA.hasChanged(hashCode())) {
      Slot0Configs gains = new Slot0Configs();
      gains.kP = flywheelKP.get();
      gains.kD = flywheelKD.get();
      gains.kS = flywheelKS.get();
      gains.kV = flywheelKV.get();
      gains.kA = flywheelKA.get();
      forwardFlywheel.setPID(gains);
      backFlywheel.setPID(gains);
    }

    if (pivotKP.hasChanged(hashCode())
        || pivotKD.hasChanged(hashCode())
        || pivotKS.hasChanged(hashCode())
        || pivotKV.hasChanged(hashCode())
        || pivotKA.hasChanged(hashCode())) {
      pivot.setPID(pivotKP.get(), 0.0, pivotKD.get(), pivotKS.get(), pivotKV.get(), pivotKA.get());
    }

    if (flywheelMaxAccelerationRotationsPerSec2.hasChanged(hashCode())
        || flywheelMaxJerkRotationsPerSec3.hasChanged(hashCode())) {
      forwardFlywheel.setMotionConstraints(
          RotationsPerSecondPerSecond.of(flywheelMaxAccelerationRotationsPerSec2.get()),
          flywheelMaxJerkRotationsPerSec3.get());
      backFlywheel.setMotionConstraints(
          RotationsPerSecondPerSecond.of(flywheelMaxAccelerationRotationsPerSec2.get()),
          flywheelMaxJerkRotationsPerSec3.get());
    }

    if (pivotMaxVelocityRotationsPerSec.hasChanged(hashCode())
        || pivotMaxAccelerationRotationsPerSec2.hasChanged(hashCode())
        || pivotMaxJerkRotationsPerSec3.hasChanged(hashCode())) {
      pivot.setMotionConstraints(
          RotationsPerSecond.of(pivotMaxVelocityRotationsPerSec.get()),
          RotationsPerSecondPerSecond.of(pivotMaxAccelerationRotationsPerSec2.get()),
          pivotMaxJerkRotationsPerSec3.get());
    }
  }

  /**
   * Gets the linear velocity of the flywheel at its effective radius.
   *
   * @return The linear velocity in meters per second.
   */
  public LinearVelocity getShooterForwardLinearVelocity() {
    return MetersPerSecond.of(
        forwardFlywheelInputs.angularVelocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters));
  }

  /**
   * runs the characterization routine.
   *
   * @param output The output voltage to apply.
   */
  private void runCharacterization(double output) {
    forwardFlywheel.setOpenLoopOutput(output);
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

  /**
   * Sets the flywheel angular velocity manually.
   *
   * @param velocity The angular velocity to set.
   */
  public void setManualVelocity(AngularVelocity velocity) {
    forwardFlywheel.setVelocity(velocity);
    backFlywheel.setVelocity(velocity);
  }

  /**
   * Checks if the shooter is ready to shoot (flywheel and pivot at setpoint).
   *
   * @return True if the shooter is ready.
   */
  @AutoLogOutput(key = "Shooter/canShoot")
  public boolean canShoot() {
    return MathUtil.isNear(
            flywheelSetpoint.in(RotationsPerSecond),
            forwardFlywheelInputs.angularVelocity.in(RotationsPerSecond),
            ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond))
        && MathUtil.isNear(
            pivotSetpoint.in(Degrees),
            pivotInputs.pivotPrimaryPosition.in(Degrees),
            ShooterConstants.pivotPositionTolerence.in(Degrees));
  }

  /**
   * Gets the current pivot angle.
   *
   * @return The pivot Angle.
   */
  public Angle getPivotAngle() {
    // return pivotInputs.pivotPrimaryPosition;
    return pivotSetpoint;
  }

  /**
   * Gets the current flywheel velocity.
   *
   * @return The flywheel AngularVelocity.
   */
  public AngularVelocity getFlywheelVelocity() {
    // return flywheelInputs.primaryVelocity;

    return flywheelSetpoint;
  }

  /**
   * Sets the shooter setpoints based on the calculated controls.
   *
   * @param controls The calculated shooter controls containing pitch and velocity.
   */
  public void setShooterControls(ShooterControls controls) {
    flywheelSetpoint = controls.shooterVelocity();
    pivotSetpoint = controls.shooterPitch();

    Logger.recordOutput(
        "Shooter/Velocity Setpoint Rotations Per Sec", flywheelSetpoint.in(RotationsPerSecond));
    Logger.recordOutput("Shooter/Pitch Setpoint Deg", pivotSetpoint.in(Degrees));

    pivot.setPosition(pivotSetpoint);
    forwardFlywheel.setVelocity(flywheelSetpoint);
    backFlywheel.setVelocity(flywheelSetpoint);
  }
}
