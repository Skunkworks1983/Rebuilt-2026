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
  BedIO bed;
  Alert hopperDisconnectAlert = new Alert(getName(), AlertType.kError);

  IndexerIO indexer;
  Alert leftIndexerDisconnectAlert = new Alert(getName(), AlertType.kError);

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

  public final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

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

  public Hopper(BedIO hopperMotor, IndexerIO indexer) {
    this.bed = hopperMotor;
    this.indexer = indexer;

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

  private void runIndexerTorqueCurrent(double torqueCurrent) {
    indexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    bed.updateInputs(hopperInputs);
    indexer.updateInputs(indexerInputs);

    Logger.processInputs("Hopper/Bed/", hopperInputs);
    Logger.processInputs("Hopper/Indexer/", indexerInputs);
    hopperDisconnectAlert.set(!hopperInputs.connected);
    leftIndexerDisconnectAlert.set(!indexerInputs.connected);

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
        indexer.setPID(gains);

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
    bed.setHopperVelocity(velocity);
  }

  public void setHopperDutyCycle(double dutyCycle) {
    bed.setDutyCycle(dutyCycle);
  }

  /** sets the velocity of the indexer to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexer.setIndexerVelocity(velocity.unaryMinus());
  }

  /** sets both indexers duty cycle */
  public void setIndexerDutyCycle(double dutyCycle) {
    indexer.setDutyCycle(dutyCycle);
  }

  public void setIndexerVoltageOut(Voltage voltage) {
    indexer.setVoltageOut(voltage.unaryMinus());
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
