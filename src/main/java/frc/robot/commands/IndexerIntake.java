// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kitIndexer.Indexer;
import frc.robot.subsystems.kitIndexer.IndexerConstants;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexerIntake extends Command {
  private final Indexer indexer;
  private final LoggedTunableNumber indexerVelocity =
      new LoggedTunableNumber(
          "Indexer/VelocitySetpointRotPerSec",
          IndexerConstants.defaultIndexerVelocity.in(RotationsPerSecond));

  /** Creates a new IndexerIntake. */
  public IndexerIntake(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setSpeed(RotationsPerSecond.of(indexerVelocity.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSpeed(RotationsPerSecond.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
