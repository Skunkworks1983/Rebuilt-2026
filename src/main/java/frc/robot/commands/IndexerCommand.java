// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;

/** Command that runs the indexer */
public class IndexerCommand extends Command {

  Indexer indexer;

  public IndexerCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // checks the drive mode and runs the indexer at the defined velocity or duty cycle
    if (IndexerConstants.indexerDriveMode.equals(IndexerConstants.IndexerDriveMode.DutyCycle)) {
      indexer.setIndexerDutyCycle(IndexerConstants.indexerDutyCycleValue);
    } else {
      indexer.setIndexerVelocity(IndexerConstants.indexerAngularVelocity);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (IndexerConstants.indexerDriveMode.equals(IndexerConstants.IndexerDriveMode.DutyCycle)) {
      indexer.setIndexerDutyCycle(0);
    } else {
      indexer.setIndexerVelocity(Units.RadiansPerSecond.of(0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
