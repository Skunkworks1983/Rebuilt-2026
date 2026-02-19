package frc.robot.subsystems.simpleShooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // FORWARD FLYWHEEL
  private final FlywheelIO forwardFlywheel;
  private final FlywheelIOInputsAutoLogged forwardFlywheelInputs = new FlywheelIOInputsAutoLogged();

  private static final LoggedTunableNumber forwardFlywheelKP =
      new LoggedTunableNumber("Shooter/UpperFlywheel/kP", ShooterConstants.upperFlywheelKp);
  private static final LoggedTunableNumber forwardFlywheelKD =
      new LoggedTunableNumber("Shooter/UpperFlywheel/kD", ShooterConstants.forwardFlywheelKD);
  private static final LoggedTunableNumber forwardFlywheelKS =
      new LoggedTunableNumber("Shooter/UpperFlywheel/kS", ShooterConstants.forwardFlywheelKS);
  private static final LoggedTunableNumber forwardFlywheelKV =
      new LoggedTunableNumber("Shooter/UpperFlywheel/kV", ShooterConstants.forwardFlywheelKV);
  private static final LoggedTunableNumber forwardFlywheelKA =
      new LoggedTunableNumber("Shooter/UpperFlywheel/kA", ShooterConstants.forwardFlywheelKA);

  private final Alert forwardFlywheelDisconnectedAlert =
      new Alert("Forward Flywheel disconnected", AlertType.kError);

  private final SysIdRoutine sysId;

  private final FlywheelIO backFlywheel;
  private final FlywheelIOInputsAutoLogged backFlywheelInputs = new FlywheelIOInputsAutoLogged();

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

  private final Alert backFlywheelDisconnectedAlert =
      new Alert("Back Flywheel disconnected", AlertType.kError);

  public Shooter(FlywheelIO forwardFlywheel, FlywheelIO backFlywheel) {
    this.forwardFlywheel = forwardFlywheel;
    this.backFlywheel = backFlywheel;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("Shooter/SysID State", state.toString())),
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
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.INFO,
              "Closed Loop Gains Updated",
              "Forward Flywheel Closed Loop Gains Updated"));
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
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.INFO,
              "Closed Loop Gains Updated",
              "Lower Flywheel Closed Loop Gains Updated"));
    }
  }

  public LinearVelocity getForwardFlywheelLinearVelocity() {
    return MetersPerSecond.of(
        forwardFlywheelInputs.velocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters));
  }

  private void runCharacterization(double output) {
    forwardFlywheel.setOpenLoopCurrent(output);
    backFlywheel.setOpenLoopCurrent(output);
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
