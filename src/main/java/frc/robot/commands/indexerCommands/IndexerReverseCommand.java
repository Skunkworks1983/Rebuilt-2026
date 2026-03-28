// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.IndexerConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the indexer */
public class IndexerReverseCommand extends Command {
  Hopper hopper;

  public IndexerReverseCommand(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setIndexerVelocity(
        RotationsPerSecond.of(-IndexerConstants.indexerAngularVelocityRotationsPerSecond.get()));
    Logger.recordOutput("Index/Reverse Command is Running", true);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setIndexerDutyCycle(0);
    Logger.recordOutput("Index/Reverse Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
