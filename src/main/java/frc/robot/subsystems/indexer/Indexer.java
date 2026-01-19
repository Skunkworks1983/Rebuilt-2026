// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  IndexerIO indexerIOMotor;
  public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerMotor) {
    this.indexerIOMotor = indexerIOMotor;
  }

  @Override
  public void periodic() {
    indexerIOMotor.updateInputs(inputs);
  }
}
