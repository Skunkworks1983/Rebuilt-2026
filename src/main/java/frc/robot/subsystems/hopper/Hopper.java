// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * {@summary} Includes the mechanisms for both the Hopper Bed and the Trigger, it takes in one
 * BedIOTalonFX and two TriggerIOTalonFX
 */
public class Hopper extends SubsystemBase {
  BedIO bed;
  Alert hopperDisconnectAlert = new Alert(getName(), AlertType.kError);

  @AutoLogOutput private AngularVelocity triggerSetpoint;

  TriggerIO trigger;
  Alert leftTriggerDisconnectAlert = new Alert(getName(), AlertType.kError);
  Alert rightTriggerDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  private static final LoggedTunableNumber hopperlKP =
      new LoggedTunableNumber("Hopper/HopperBed/kP", BedConstants.hopperKP);
  private static final LoggedTunableNumber hopperlKD =
      new LoggedTunableNumber("Hopper/HopperBed/kD", BedConstants.hopperKD);
  private static final LoggedTunableNumber hopperlKS =
      new LoggedTunableNumber("Hopper/HopperBed/kS", BedConstants.hopperKS);
  private static final LoggedTunableNumber hopperlKV =
      new LoggedTunableNumber("Hopper/HopperBed/kV", BedConstants.hopperKV);
  private static final LoggedTunableNumber hopperlKA =
      new LoggedTunableNumber("Hopper/HopperBed/kA", BedConstants.hopperKA);

  public final TriggerIOInputsAutoLogged triggerInputs = new TriggerIOInputsAutoLogged();

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

  public Hopper(BedIO hopperMotor, TriggerIO trigger) {
    this.bed = hopperMotor;
    this.trigger = trigger;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                BedConstants.sysIdRampRate,
                BedConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runHopperTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runHopperTorqueCurrent(double torqueCurrent) {
    bed.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    bed.updateInputs(hopperInputs);
    trigger.updateInputs(triggerInputs);

    Logger.processInputs("Hopper/Bed/", hopperInputs);
    Logger.processInputs("Hopper/Trigger/", triggerInputs);
    hopperDisconnectAlert.set(!hopperInputs.connected);
    leftTriggerDisconnectAlert.set(!triggerInputs.primaryConnected);
    rightTriggerDisconnectAlert.set(!triggerInputs.followerConnected);

    if (Constants.tuningMode) {

      int hash = hashCode();
      if (hopperlKP.hasChanged(hash)
          || hopperlKD.hasChanged(hash)
          || hopperlKS.hasChanged(hash)
          || hopperlKV.hasChanged(hash)
          || hopperlKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = hopperlKP.get();
        gains.kD = hopperlKD.get();
        gains.kS = hopperlKS.get();
        gains.kV = hopperlKV.get();
        gains.kA = hopperlKA.get();
        bed.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Hopper PID Update",
                "Hopper motor PID gains have been updated."));
      }

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
        trigger.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Trigger PID Update",
                "Trigger motor PID gains have been updated."));
      }
    }
  }

  public void setHopperDutyCycle(double dutyCycle) {
    bed.setDutyCycle(dutyCycle);
  }

  /** sets the velocity of the trigger to the defined velocity */
  public void setTriggerVelocity(AngularVelocity velocity) {
    triggerSetpoint = velocity.unaryMinus();
    trigger.setTriggerVelocity(velocity.unaryMinus());
  }

  /** sets both triggers duty cycle */
  public void setTriggerDutyCycle(double dutyCycle) {
    trigger.setDutyCycle(dutyCycle);
  }

  public void setTriggerVoltageOut(Voltage voltage) {
    trigger.setVoltageOut(voltage.unaryMinus());
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
