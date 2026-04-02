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
  HopperIO hopperBed;
  Alert hopperBedDisconnectAlert = new Alert(getName(), AlertType.kError);

  IndexerIO leftIndexer;
  Alert leftIndexerDisconnectAlert = new Alert(getName(), AlertType.kError);

  IndexerIO rightIndexer;
  Alert rightIndexerDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final HopperIOInputsAutoLogged hopperBedInputs = new HopperIOInputsAutoLogged();

  private static final LoggedTunableNumber hopperBedlKP =
      new LoggedTunableNumber("Hopper/HopperBed/kP", HopperBedConstants.hopperKP);
  private static final LoggedTunableNumber hopperBedlKD =
      new LoggedTunableNumber("Hopper/HopperBed/kD", HopperBedConstants.hopperKD);
  private static final LoggedTunableNumber hopperBedlKS =
      new LoggedTunableNumber("Hopper/HopperBed/kS", HopperBedConstants.hopperKS);
  private static final LoggedTunableNumber hopperBedlKV =
      new LoggedTunableNumber("Hopper/HopperBed/kV", HopperBedConstants.hopperKV);
  private static final LoggedTunableNumber hopperBedlKA =
      new LoggedTunableNumber("Hopper/HopperBed/kA", HopperBedConstants.hopperKA);

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

  public Hopper(HopperIO hopperBedMotor, IndexerIO leftIndexerMotor, IndexerIO rightIndexerMotor) {
    this.hopperBed = hopperBedMotor;
    this.leftIndexer = leftIndexerMotor;
    this.rightIndexer = rightIndexerMotor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                HopperBedConstants.sysIdRampRate,
                HopperBedConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runHopperTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runHopperTorqueCurrent(double torqueCurrent) {
    hopperBed.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  private void runIndexerTorqueCurrent(double torqueCurrent) {
    leftIndexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
    rightIndexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    hopperBed.updateInputs(hopperBedInputs);
    leftIndexer.updateInputs(leftIndexerInputs);
    rightIndexer.updateInputs(rightIndexerInputs);

    Logger.processInputs("Hopper/HopperBed/Hopper Motor", hopperBedInputs);
    Logger.processInputs("Hopper/Indexer/Left Indexer", leftIndexerInputs);
    Logger.processInputs("Hopper/Indexer/Right Indexer", rightIndexerInputs);
    hopperBedDisconnectAlert.set(!hopperBedInputs.connected);
    leftIndexerDisconnectAlert.set(!leftIndexerInputs.connected);
    rightIndexerDisconnectAlert.set(!rightIndexerInputs.connected);

    if (Constants.tuningMode) {

      int hash = hashCode();
      if (hopperBedlKP.hasChanged(hash)
          || hopperBedlKD.hasChanged(hash)
          || hopperBedlKS.hasChanged(hash)
          || hopperBedlKV.hasChanged(hash)
          || hopperBedlKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = hopperBedlKP.get();
        gains.kD = hopperBedlKD.get();
        gains.kS = hopperBedlKS.get();
        gains.kV = hopperBedlKV.get();
        gains.kA = hopperBedlKA.get();
        hopperBed.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Hopper Bed PID Update",
                "Hopper Bed motor PID gains have been updated."));
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
  public void setHopperBedVelocity(AngularVelocity velocity) {
    hopperBed.setHopperVelocity(velocity);
  }

  public void setHopperBedDutyCycle(double dutyCycle) {
    hopperBed.setDutyCycle(dutyCycle);
  }

  /** sets the velocity of the indexer to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    leftIndexer.setIndexerVelocity(velocity.unaryMinus());
    rightIndexer.setIndexerVelocity(velocity);
  }

  /** sets both indexers duty cycle */
  public void setIndexerDutyCycle(double dutyCycle) {
    leftIndexer.setDutyCycle(dutyCycle);
    rightIndexer.setDutyCycle(-dutyCycle);
  }

  public void setIndexerVoltageOut(Voltage voltage) {
    leftIndexer.setVoltageOut(voltage.unaryMinus());
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
