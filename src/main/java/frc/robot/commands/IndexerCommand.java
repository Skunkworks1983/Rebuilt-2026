// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommand extends Command {

  Indexer indexer;

  public IndexerCommand(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void initialize() {
    indexer.setIndexerVelocity(2);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
