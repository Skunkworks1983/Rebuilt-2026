// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerAutoUnjam extends Command {
  private static final String logKey = "Index/AutoUnjam";

  private enum AutoUnjamState {
    FEED,
    REVERSE
  }

  private final Indexer indexer;
  private final Hopper hopper;
  private final Timer timer = new Timer();
  private final BooleanSupplier beamBroken;

  private AutoUnjamState state = AutoUnjamState.FEED;

  public IndexerAutoUnjam(
      Indexer indexer,
      BooleanSupplier beamBroken,
      Hopper hopper) {
    this.indexer = indexer;
    this.beamBroken = beamBroken;
    this.hopper = hopper;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    Logger.recordOutput(logKey + "/Running", true);
    Logger.recordOutput(logKey + "Index/Unjamming", false);
    state = AutoUnjamState.FEED;
    timer.restart();
    applyStateOutput();
  }

  @Override
  public void execute() {
    switch (state) {
      case FEED:
        if (!beamBroken.getAsBoolean()) {
          timer.restart();
        } else if (timer.hasElapsed(IndexerConstants.autoUnjamNoBeamBreakTimeoutSeconds.get())) {
          setState(AutoUnjamState.REVERSE);
        }
        break;
      case REVERSE:
        if (timer.hasElapsed(IndexerConstants.autoUnjamReverseDurationSeconds.get())) {
          setState(AutoUnjamState.FEED);
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    indexer.setIndexerDutyCycle(0.0);
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput(logKey + "/Running", false);
    Logger.recordOutput(logKey + "/Unjamming", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void setState(AutoUnjamState nextState) {
    // if (state == nextState) {
    //   return;
    // }

    state = nextState;
    timer.restart();
    applyStateOutput();
  }

  private void applyStateOutput() {
    double indexerSpeedRps = IndexerConstants.indexerAngularVelocityRotationsPerSecond.get();
    double hopperSpeedRps = HopperConstants.hopperAngularVelocityRotationsPerSecond.get();
    double hopperReverseSpeedRps =
        HopperConstants.hopperReverseAngularVelocityRotationsPerSecond.get();

    if (state == AutoUnjamState.FEED) {
      indexer.setIndexerVelocity(RotationsPerSecond.of(indexerSpeedRps));
      hopper.setHopperVelocity(RotationsPerSecond.of(hopperSpeedRps));
      Logger.recordOutput(logKey + "/Unjamming", false);
    } else {
      indexer.setIndexerVelocity(RotationsPerSecond.of(-indexerSpeedRps));
      hopper.setHopperVelocity(RotationsPerSecond.of(hopperReverseSpeedRps));
      Logger.recordOutput(logKey + "/Unjamming", true);
    }
  }
}
