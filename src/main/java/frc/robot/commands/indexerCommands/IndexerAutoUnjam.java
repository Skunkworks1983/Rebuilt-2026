// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperBedConstants;
import frc.robot.subsystems.hopper.IndexerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerAutoUnjam extends Command {
  private static final String logKey = "Index/AutoUnjam";

  private enum AutoUnjamState {
    FEED,
    REVERSE
  }

  private final Hopper hopper;
  private final Timer timer = new Timer();
  private final BooleanSupplier beamBroken;

  private AutoUnjamState state = AutoUnjamState.FEED;

  public IndexerAutoUnjam(BooleanSupplier beamBroken, Hopper hopper) {
    this.beamBroken = beamBroken;
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    // hopper.setHopperBrake();
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
    hopper.setIndexerDutyCycle(0.0);
    Logger.recordOutput(logKey + "/Running", false);
    Logger.recordOutput(logKey + "/Unjamming", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void setState(AutoUnjamState nextState) {
    if (state == nextState) {
      return;
    }

    state = nextState;
    timer.restart();
    applyStateOutput();
  }

  private void applyStateOutput() {
    double indexerSpeedRps = IndexerConstants.indexerAngularVelocityRotationsPerSecond.get();
    double hopperReverseSpeedRps =
        HopperBedConstants.hopperReverseAngularVelocityRotationsPerSecond.get();

    if (state == AutoUnjamState.FEED) {
      hopper.setIndexerDutyCycle(IndexerConstants.indexerDutyCycleSpeedFeed.get());
      Logger.recordOutput(logKey + "/Unjamming", false);
    } else {
      hopper.setIndexerDutyCycle(IndexerConstants.indexerDutyCycleSpeedReverse.get());
      Logger.recordOutput(logKey + "/Unjamming", true);
    }
  }
}
