// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the indexer */
public class IndexerCommand extends Command {
  Indexer indexer;

  public IndexerCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setIndexerVelocity(
        RotationsPerSecond.of(IndexerConstants.indexerAngularVelocityRotationsPerSecond.get()));
    Logger.recordOutput("Index/Command is Running", true);
  }

  @Override
  public void execute() {
    // indexer.setIndexerDutyCycle(0.7);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerDutyCycle(0);
    Logger.recordOutput("Index/Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
