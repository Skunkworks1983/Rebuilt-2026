// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.TriggerConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the trigger */
public class TriggerCommand extends Command {
  Hopper hopper;

  public TriggerCommand(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setTriggerDutyCycle(TriggerConstants.triggerDutyCycleSpeedFeed.get());
    Logger.recordOutput("Trigger/Command is Running", true);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    Logger.recordOutput("Trigger/Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
