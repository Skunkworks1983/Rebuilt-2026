// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.TriggerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class HopperAutoUnjam extends Command {
  private static final String logKey = "Trigger/AutoUnjam";

  private enum AutoUnjamState {
    FEED,
    REVERSE
  }

  private final Hopper hopper;
  private final Timer timer = new Timer();
  private final BooleanSupplier beamBroken;

  private AutoUnjamState state = AutoUnjamState.FEED;

  public HopperAutoUnjam(BooleanSupplier beamBroken, Hopper hopper) {
    this.beamBroken = beamBroken;
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    Logger.recordOutput(logKey + "/Running", true);
    Logger.recordOutput(logKey + "/Unjamming", false);
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
        } else if (timer.hasElapsed(TriggerConstants.autoUnjamNoBeamBreakTimeoutSeconds.get())) {
          setState(AutoUnjamState.REVERSE);
        }
        break;
      case REVERSE:
        if (timer.hasElapsed(TriggerConstants.autoUnjamReverseDurationSeconds.get())) {
          setState(AutoUnjamState.FEED);
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    hopper.setTriggerDutyCycle(0.0);
    hopper.setHopperDutyCycle(0.0);
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
    if (state == AutoUnjamState.FEED) {
      hopper.setTriggerVelocity(
          RotationsPerSecond.of(TriggerConstants.triggerAngularVelocityRotationsPerSecond.get()));
      hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
      Logger.recordOutput(logKey + "/Unjamming", false);
    } else {
      hopper.setTriggerVelocity(
          RotationsPerSecond.of(
              TriggerConstants.triggerAngularVelocityRotationsPerSecond.get() * -1));
      hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedReverse.get());
      Logger.recordOutput(logKey + "/Unjamming", true);
    }
  }
}
