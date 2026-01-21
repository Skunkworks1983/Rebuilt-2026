// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * {@summary} Requires one IndexerIOTalonFX
 *
 * @author Cyrus M.
 */
public class Indexer extends SubsystemBase {

  IndexerIO indexerMotor;
  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerMotor) {
    this.indexerMotor = indexerMotor;
  }

  @Override
  public void periodic() {
    indexerMotor.updateInputs(inputs);
  }

  public void setIndexerVelocity(double VelocityDegPerSec) {
    indexerMotor.setIndexerVelocity(VelocityDegPerSec);
  }
}
