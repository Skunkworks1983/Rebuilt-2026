// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * {@summary} Requires one IndexerIOTalonFX and can run said motor at a defined velocity
 *
 * @author Cyrus M.
 */
public class Indexer extends SubsystemBase {

  IndexerIO indexerMotor;
  Alert indexerDisconnectAlert = new Alert(getName(), AlertType.kError);
  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerMotor) {
    this.indexerMotor = indexerMotor;
  }

  @Override
  public void periodic() {
    indexerMotor.updateInputs(inputs);
    Logger.processInputs("Indexer/Indexer Motor", inputs);
    indexerDisconnectAlert.set(!inputs.connected);
  }

  /** sets the velocity of the indexer to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerMotor.setIndexerVelocity(velocity);
  }

  /** sets the duty cycle of the indexer to the defined value */
  public void setIndexerDutyCycle(double dutyCycle) {
    indexerMotor.setIndexerDutyCycle(dutyCycle);
  }
}
