// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

/** Command that runs the indexer */
public class IndexerCommand extends Command {

  Indexer indexer;

  public IndexerCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setIndexerVelocity(Constants.IndexerConstants.indexerAngularVelocity);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerVelocity(Units.RotationsPerSecond.of(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
