// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.TriggerConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the trigger */
public class RunTrigger extends Command {
  Hopper hopper;

  public RunTrigger(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setTriggerVelocity(
        RotationsPerSecond.of(TriggerConstants.triggerAngularVelocityRotationsPerSecond.get()));
    hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
    Logger.recordOutput("Trigger/Run Trigger Command is Running", true);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    Logger.recordOutput("Trigger/Run Trigger Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
