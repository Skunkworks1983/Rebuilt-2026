package frc.robot.subsystems.simpleShooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum ShooterState {
    OFF,
    SHOOT_GOAL,
    SHOOT_ALLIANCE_ZONE
  }

  private ShooterState shooterState = ShooterState.OFF;

  // FORWARD FLYWHEEL
  private final FlywheelIO forwardFlywheel;
  private final FlywheelIOInputsAutoLogged forwardFlywheelInputs = new FlywheelIOInputsAutoLogged();

  private AngularVelocity forwardFlywheelSetpoint = RadiansPerSecond.of(0 + .1e-6);

  private static final LoggedTunableNumber forwardFlywheelKP =
      new LoggedTunableNumber("Shooter/ForwardFlywheel/kP", ShooterConstants.forwardFlywheelKP);
  private static final LoggedTunableNumber forwardFlywheelKD =
      new LoggedTunableNumber("Shooter/ForwardFlywheel/kD", ShooterConstants.forwardFlywheelKD);
  private static final LoggedTunableNumber forwardFlywheelKS =
      new LoggedTunableNumber("Shooter/ForwardFlywheel/kS", ShooterConstants.forwardFlywheelKS);
  private static final LoggedTunableNumber forwardFlywheelKV =
      new LoggedTunableNumber("Shooter/ForwardFlywheel/kV", ShooterConstants.forwardFlywheelKV);
  private static final LoggedTunableNumber forwardFlywheelKA =
      new LoggedTunableNumber("Shooter/ForwardFlywheel/kA", ShooterConstants.forwardFlywheelKA);

  private static final LoggedTunableNumber forwardFlywheelMaxAccelerationRotationsPerSec2 =
      new LoggedTunableNumber(
          "Shooter/ForwardFlywheel/MaxAccelerationRotationsPerSec2",
          ShooterConstants.forwardFlywheelMaxAccelerationRotationsPerSec);
  private static final LoggedTunableNumber forwardFlywheelMaxJerkRotationsPerSec3 =
      new LoggedTunableNumber(
          "Shooter/ForwardFlywheel/MaxJerkRotationsPerSec3",
          ShooterConstants.forwardFlywheelMaxJerkRotationsPerSecCubed);

  private final Alert forwardFlywheelDisconnectedAlert =
      new Alert("Forward Flywheel disconnected", AlertType.kError);

  private final SysIdRoutine sysId;

  private final FlywheelIO backFlywheel;
  private final FlywheelIOInputsAutoLogged backFlywheelInputs = new FlywheelIOInputsAutoLogged();

  private AngularVelocity backFlywheelSetpoint = RadiansPerSecond.of(0 + .1e-6);

  private static final LoggedTunableNumber backFlywheelKP =
      new LoggedTunableNumber("Shooter/BackFlywheel/kP", ShooterConstants.backFlywheelKP);
  private static final LoggedTunableNumber backFlywheelKD =
      new LoggedTunableNumber("Shooter/BackFlywheel/kD", ShooterConstants.backFlywheelKD);
  private static final LoggedTunableNumber backFlywheelKS =
      new LoggedTunableNumber("Shooter/BackFlywheel/kS", ShooterConstants.backFlywheelKS);
  private static final LoggedTunableNumber backFlywheelKV =
      new LoggedTunableNumber("Shooter/BackFlywheel/kV", ShooterConstants.backFlywheelKV);
  private static final LoggedTunableNumber backFlywheelKA =
      new LoggedTunableNumber("Shooter/BackFlywheel/kA", ShooterConstants.backFlywheelKA);

  private static final LoggedTunableNumber backFlywheelMaxAccelerationRotationsPerSec2 =
      new LoggedTunableNumber(
          "Shooter/BackFlywheel/MaxAccelerationRotationsPerSec2",
          ShooterConstants.backFlywheelMaxAccelerationRotationsPerSec);
  private static final LoggedTunableNumber backFlywheelMaxJerkRotationsPerSec3 =
      new LoggedTunableNumber(
          "Shooter/BackFlywheel/MaxJerkRotationsPerSec3",
          ShooterConstants.backFlywheelMaxJerkRotationsPerSecCubed);

  private final Alert backFlywheelDisconnectedAlert =
      new Alert("Back Flywheel disconnected", AlertType.kError);

  public Shooter(FlywheelIO forwardFlywheel, FlywheelIO backFlywheel) {
    this.forwardFlywheel = forwardFlywheel;
    this.backFlywheel = backFlywheel;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(2),
                Volts.of(5),
                null,
                (state) -> Logger.recordOutput("Shooter/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    forwardFlywheel.updateInputs(forwardFlywheelInputs);
    Logger.processInputs("Shooter/Upper (Launch) Flywheel", forwardFlywheelInputs);

    forwardFlywheelDisconnectedAlert.set(
        (!forwardFlywheelInputs.primaryConnected) && Constants.currentMode != Constants.Mode.SIM);

    if (forwardFlywheelKP.hasChanged(hashCode())
        || forwardFlywheelKD.hasChanged(hashCode())
        || forwardFlywheelKS.hasChanged(hashCode())
        || forwardFlywheelKV.hasChanged(hashCode())
        || forwardFlywheelKA.hasChanged(hashCode())) {
      forwardFlywheel.setPID(
          forwardFlywheelKP.get(),
          0.0,
          forwardFlywheelKD.get(),
          forwardFlywheelKS.get(),
          forwardFlywheelKV.get(),
          forwardFlywheelKA.get());
    }

    if (forwardFlywheelMaxAccelerationRotationsPerSec2.hasChanged(hashCode())
        || forwardFlywheelMaxJerkRotationsPerSec3.hasChanged(hashCode())) {
      forwardFlywheel.setMotionConstraints(
          RotationsPerSecondPerSecond.of(forwardFlywheelMaxAccelerationRotationsPerSec2.get()),
          forwardFlywheelMaxAccelerationRotationsPerSec2.get());
    }

    backFlywheel.updateInputs(backFlywheelInputs);
    Logger.processInputs("Shooter/Lower Flywheel", backFlywheelInputs);

    backFlywheelDisconnectedAlert.set(
        (!forwardFlywheelInputs.primaryConnected) && Constants.currentMode != Constants.Mode.SIM);

    if (backFlywheelKP.hasChanged(hashCode())
        || backFlywheelKD.hasChanged(hashCode())
        || backFlywheelKS.hasChanged(hashCode())
        || backFlywheelKV.hasChanged(hashCode())
        || backFlywheelKA.hasChanged(hashCode())) {
      backFlywheel.setPID(
          backFlywheelKP.get(),
          0.0,
          backFlywheelKD.get(),
          backFlywheelKS.get(),
          backFlywheelKV.get(),
          backFlywheelKA.get());
    }

    if (backFlywheelMaxAccelerationRotationsPerSec2.hasChanged(hashCode())
        || backFlywheelMaxJerkRotationsPerSec3.hasChanged(hashCode())) {
      backFlywheel.setMotionConstraints(
          RotationsPerSecondPerSecond.of(backFlywheelMaxAccelerationRotationsPerSec2.get()),
          backFlywheelMaxAccelerationRotationsPerSec2.get());
    }
  }

  public LinearVelocity getForwardFlywheelLinearVelocity() {
    return MetersPerSecond.of(
        forwardFlywheelInputs.velocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters));
  }

  private void runCharacterization(double output) {
    forwardFlywheel.setOpenLoopOutput(output);
    backFlywheel.setOpenLoopOutput(output);
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

  @AutoLogOutput(key = "Shooter/canShoot")
  public boolean canShoot() {
    return true;
  }

  public AngularVelocity getForwardFlywheelVelocity() {
    return forwardFlywheelInputs.velocity;
  }

  public void setShooterControls(
      AngularVelocity forwardFlywheelVelocity, AngularVelocity backFlywheelVelocity) {
    Logger.recordOutput(
        "Shooter/ForwardFlywheel/Velocity Setpoint Rotations Per Sec",
        forwardFlywheelVelocity.in(RotationsPerSecond));
    Logger.recordOutput(
        "Shooter/BackFlywheel/Velocity Setpoint Rotations Per Sec",
        backFlywheelVelocity.in(RotationsPerSecond));

    forwardFlywheel.setVelocity(forwardFlywheelVelocity);
    backFlywheel.setVelocity(backFlywheelVelocity);
  }

  public void maxShooterDuty() {
    forwardFlywheel.maxDutyCycle();
    backFlywheel.maxDutyCycle();
  }
}
