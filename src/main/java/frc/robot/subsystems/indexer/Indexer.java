// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * {@summary} Requires one IndexerIOTalonFX and can run said motor at a defined velocity
 *
 * @author Cyrus M.
 */
public class Indexer extends SubsystemBase {
  IndexerIO indexer;
  Alert indexerDisconnectAlert = new Alert(getName(), AlertType.kError);

  private final SysIdRoutine sysId;

  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private static final LoggedTunableNumber indexerlKP =
      new LoggedTunableNumber("Indexer/kP", IndexerConstants.indexerKP);
  private static final LoggedTunableNumber indexerlKD =
      new LoggedTunableNumber("Indexer/kD", IndexerConstants.indexerKD);
  private static final LoggedTunableNumber indexerlKS =
      new LoggedTunableNumber("Indexer/kS", IndexerConstants.indexerKS);
  private static final LoggedTunableNumber indexerlKV =
      new LoggedTunableNumber("Indexer/kV", IndexerConstants.indexerKV);
  private static final LoggedTunableNumber indexerlKA =
      new LoggedTunableNumber("Indexer/kA", IndexerConstants.indexerKA);

  public Indexer(IndexerIO indexerMotor) {
    this.indexer = indexerMotor;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                IndexerConstants.sysIdRampRate,
                IndexerConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Indexer/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runIndexerTorqueCurrent(voltage.in(Volts)), null, this));
  }

  private void runIndexerTorqueCurrent(double torqueCurrent) {
    indexer.setOpenLoopTorqueCurrent(Amps.of(torqueCurrent));
  }

  @Override
  public void periodic() {
    indexer.updateInputs(inputs);
    Logger.processInputs("Indexer/Indexer Motor", inputs);
    indexerDisconnectAlert.set(!inputs.connected);

    if (indexerlKP.hasChanged(hashCode())
        || indexerlKD.hasChanged(hashCode())
        || indexerlKS.hasChanged(hashCode())
        || indexerlKV.hasChanged(hashCode())
        || indexerlKA.hasChanged(hashCode())) {
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

  /** sets the velocity of the indexer to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexer.setIndexerVelocity(velocity);
  }

  public void setIndexerDutyCycle(double dutyCycle) {
    indexer.setDutyCycle(dutyCycle);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runIndexerTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runIndexerTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }
}
