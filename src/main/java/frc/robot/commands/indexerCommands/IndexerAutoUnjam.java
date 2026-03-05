// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class IndexerAutoUnjam extends Command {
  Indexer indexer;
  Timer timer;
  BooleanSupplier beamBroken;

  private LoggedTunableNumber indexerAngularVelocity =
      new LoggedTunableNumber(
          "Indexer/AngularVelocity [Rot/s]",
          IndexerConstants.indexerAngularVelocity.in(Units.RotationsPerSecond));

  public IndexerAutoUnjam(Indexer indexer, BooleanSupplier beamBroken) {
    this.indexer = indexer;
    this.beamBroken = beamBroken;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Index/Auto-Unjam Command is Running", true);
    timer.start();
    indexer.setIndexerVelocity(RotationsPerSecond.of(indexerAngularVelocity.get()));
  }

  @Override
  public void execute() {

    if (beamBroken.getAsBoolean()) {
      timer.reset();
      return;
    }

    if (timer.get() < 1) {
      indexer.setIndexerVelocity(RotationsPerSecond.of(indexerAngularVelocity.get()));
      Logger.recordOutput("Index/Reverse Command is Running", false);
      Logger.recordOutput("Index/Command is Running", true);
    } else if(timer.get() < 2){
      indexer.setIndexerVelocity(RotationsPerSecond.of(indexerAngularVelocity.get() * -1));
      Logger.recordOutput("Index/Command is Running", false);
      Logger.recordOutput("Index/Reverse Command is Running", true);
    } else {
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerDutyCycle(0);
    Logger.recordOutput("Index/Auto-Unjam Command is Running", false);
    Logger.recordOutput("Index/Command is Running", false);
    Logger.recordOutput("Index/Reverse Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
