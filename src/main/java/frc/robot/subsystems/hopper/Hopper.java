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
import org.littletonrobotics.junction.Logger;

/**
 * {@summary} Includes the mechanisms for both the Hopper Bed and the Indexer, it takes in one
 * Hopper motor and two Indexer motors
 */
public class Hopper extends SubsystemBase {
  HopperIO hopper;
  Alert hopperDisconnectAlert = new Alert(getName(), AlertType.kError);

  IndexerIO leftIndexer;
  Alert leftIndexerDisconnectAlert = new Alert(getName(), AlertType.kError);

  IndexerIO rightIndexer;
  Alert rightIndexerDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  private static final LoggedTunableNumber hopperlKP =
      new LoggedTunableNumber("Hopper/HopperBed/kP", HopperConstants.hopperKP);
  private static final LoggedTunableNumber hopperlKD =
      new LoggedTunableNumber("Hopper/HopperBed/kD", HopperConstants.hopperKD);
  private static final LoggedTunableNumber hopperlKS =
      new LoggedTunableNumber("Hopper/HopperBed/kS", HopperConstants.hopperKS);
  private static final LoggedTunableNumber hopperlKV =
      new LoggedTunableNumber("Hopper/HopperBed/kV", HopperConstants.hopperKV);
  private static final LoggedTunableNumber hopperlKA =
      new LoggedTunableNumber("Hopper/HopperBed/kA", HopperConstants.hopperKA);

  public final IndexerIOInputsAutoLogged leftIndexerInputs = new IndexerIOInputsAutoLogged();
  public final IndexerIOInputsAutoLogged rightIndexerInputs = new IndexerIOInputsAutoLogged();

  private static final LoggedTunableNumber indexerlKP =
      new LoggedTunableNumber("Hopper/Indexer/kP", IndexerConstants.indexerKP);
  private static final LoggedTunableNumber indexerlKD =
      new LoggedTunableNumber("Hopper/Indexer/kD", IndexerConstants.indexerKD);
  private static final LoggedTunableNumber indexerlKS =
      new LoggedTunableNumber("Hopper/Indexer/kS", IndexerConstants.indexerKS);
  private static final LoggedTunableNumber indexerlKV =
      new LoggedTunableNumber("Hopper/Indexer/kV", IndexerConstants.indexerKV);
  private static final LoggedTunableNumber indexerlKA =
      new LoggedTunableNumber("Hopper/Indexer/kA", IndexerConstants.indexerKA);

  public Hopper(HopperIO hopperMotor, IndexerIO leftIndexerMotor, IndexerIO rightIndexerMotor) {
    this.hopper = hopperMotor;
    this.leftIndexer = leftIndexerMotor;
    this.rightIndexer = rightIndexerMotor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                HopperConstants.sysIdRampRate,
                HopperConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runHopperTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runHopperTorqueCurrent(double torqueCurrent) {
    hopper.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  private void runIndexerTorqueCurrent(double torqueCurrent) {
    leftIndexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
    rightIndexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    hopper.updateInputs(hopperInputs);
    leftIndexer.updateInputs(leftIndexerInputs);
    rightIndexer.updateInputs(rightIndexerInputs);

    Logger.processInputs("Hopper/HopperBed/Hopper Motor", hopperInputs);
    Logger.processInputs("Hopper/Indexer/Left Indexer", leftIndexerInputs);
    Logger.processInputs("Hopper/Indexer/Right Indexer", rightIndexerInputs);
    hopperDisconnectAlert.set(!hopperInputs.connected);
    leftIndexerDisconnectAlert.set(!leftIndexerInputs.connected);
    rightIndexerDisconnectAlert.set(!rightIndexerInputs.connected);

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
        hopper.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Hopper PID Update",
                "Hopper motor PID gains have been updated."));
      }

      if (indexerlKP.hasChanged(hash)
          || indexerlKD.hasChanged(hash)
          || indexerlKS.hasChanged(hash)
          || indexerlKV.hasChanged(hash)
          || indexerlKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = indexerlKP.get();
        gains.kD = indexerlKD.get();
        gains.kS = indexerlKS.get();
        gains.kV = indexerlKV.get();
        gains.kA = indexerlKA.get();
        leftIndexer.setPID(gains);
        rightIndexer.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Indexer PID Update",
                "Indexer motor PID gains have been updated."));
      }
    }
  }

  /** sets the velocity of the hopper to the defined velocity */
  public void setHopperVelocity(AngularVelocity velocity) {
    hopper.setHopperVelocity(velocity);
  }

  public void setHopperBrake() {
    hopper.setHopperBrake();
  }

  public void setHopperDutyCycle(double dutyCycle) {
    hopper.setDutyCycle(dutyCycle);
  }

  /** sets the velocity of the indexer to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    leftIndexer.setIndexerVelocity(velocity);
    rightIndexer.setIndexerVelocity(velocity);
  }

  /** sets */
  public void setIndexerDutyCycle(double dutyCycle) {
    leftIndexer.setDutyCycle(dutyCycle);
    rightIndexer.setDutyCycle(dutyCycle);
  }

  public void setIndexerVoltageOut(Voltage voltage) {
    leftIndexer.setVoltageOut(voltage);
    rightIndexer.setVoltageOut(voltage);
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
