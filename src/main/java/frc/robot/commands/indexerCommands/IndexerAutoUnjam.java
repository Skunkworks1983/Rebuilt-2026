// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import java.util.function.BooleanSupplier;

public class IndexerAutoUnjam extends Command {
  Indexer indexer;
  Timer timer;
  BooleanSupplier beamBroken;
  IndexerReverseCommand indexerReverseCommand;
  IndexerCommand indexerCommand;

  public IndexerAutoUnjam(Indexer indexer, BooleanSupplier beamBroken) {
    this.indexer = indexer;
    this.beamBroken = beamBroken;
    timer = new Timer();
    indexerReverseCommand = new IndexerReverseCommand(indexer);
    indexerCommand = new IndexerCommand(indexer);

  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {

    if (beamBroken.getAsBoolean() == true) {
      timer.reset();
      return;
    }

    if (timer.get() > 1) {
      CommandScheduler.getInstance().cancel(indexerCommand);
      CommandScheduler.getInstance().schedule(indexerReverseCommand);
    } else {
      CommandScheduler.getInstance().cancel(indexerReverseCommand);
      CommandScheduler.getInstance().schedule(indexerCommand);
    }

    if (timer.get() > 2) {
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(indexerCommand);
    CommandScheduler.getInstance().cancel(indexerReverseCommand);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
