// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import java.util.function.BooleanSupplier;

public class IndexerAutoUnjam extends Command {
  Indexer indexer;
  Timer timer;
  BooleanSupplier beamBroken;

  public IndexerAutoUnjam(Indexer indexer, BooleanSupplier beamBroken) {
    this.indexer = indexer;
    this.beamBroken = beamBroken;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    timer.start();

    if (beamBroken.getAsBoolean() == true) {
      timer.reset();
    }

    if (timer.get() > 1) {
      new IndexerReverseCommand(indexer);
    } else {
      new IndexerCommand(indexer);
    }

    if (timer.get() > 2) {
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
