// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * {@summary} Includes the spinning indexing wheels, it takes in two Trigger motor IO's
 */
public class Hopper extends SubsystemBase {

  TriggerIO leftTrigger;
  Alert leftTriggerDisconnectAlert = new Alert(getName(), AlertType.kError);

  TriggerIO rightTrigger;
  Alert rightTriggerDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final TriggerIOInputsAutoLogged leftTriggerInputs = new TriggerIOInputsAutoLogged();
  public final TriggerIOInputsAutoLogged rightTriggerInputs = new TriggerIOInputsAutoLogged();

  private static final LoggedTunableNumber triggerlKP =
      new LoggedTunableNumber("Hopper/Trigger/kP", TriggerConstants.triggerKP);
  private static final LoggedTunableNumber triggerlKD =
      new LoggedTunableNumber("Hopper/Trigger/kD", TriggerConstants.triggerKD);
  private static final LoggedTunableNumber triggerlKS =
      new LoggedTunableNumber("Hopper/Trigger/kS", TriggerConstants.triggerKS);
  private static final LoggedTunableNumber triggerlKV =
      new LoggedTunableNumber("Hopper/Trigger/kV", TriggerConstants.triggerKV);
  private static final LoggedTunableNumber triggerlKA =
      new LoggedTunableNumber("Hopper/Trigger/kA", TriggerConstants.triggerKA);

  public Hopper(TriggerIO leftTriggerMotor, TriggerIO rightTriggerMotor) {
    this.leftTrigger = leftTriggerMotor;
    this.rightTrigger = rightTriggerMotor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                TriggerConstants.sysIdRampRate,
                TriggerConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runHopperTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runHopperTorqueCurrent(double torqueCurrent) {
    leftTrigger.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
    rightTrigger.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    leftTrigger.updateInputs(leftTriggerInputs);
    rightTrigger.updateInputs(rightTriggerInputs);

    Logger.processInputs("Hopper/Trigger/Left Trigger", leftTriggerInputs);
    Logger.processInputs("Hopper/Trigger/Right Trigger", rightTriggerInputs);
    leftTriggerDisconnectAlert.set(!leftTriggerInputs.connected);
    rightTriggerDisconnectAlert.set(!rightTriggerInputs.connected);

    if (Constants.tuningMode) {

      int hash = hashCode();

      if (triggerlKP.hasChanged(hash)
          || triggerlKD.hasChanged(hash)
          || triggerlKS.hasChanged(hash)
          || triggerlKV.hasChanged(hash)
          || triggerlKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = triggerlKP.get();
        gains.kD = triggerlKD.get();
        gains.kS = triggerlKS.get();
        gains.kV = triggerlKV.get();
        gains.kA = triggerlKA.get();
        leftTrigger.setPID(gains);
        rightTrigger.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Trigger PID Update",
                "Trigger motor PID gains have been updated."));
      }
    }
  }

  /** sets both triggers duty cycle */
  public void setTriggerDutyCycle(double dutyCycle) {
    leftTrigger.setDutyCycle(dutyCycle);
    rightTrigger.setDutyCycle(-dutyCycle);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runHopperTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runHopperTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }
}
