// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.TriggerConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the trigger in reveres */
public class ReverseIndexCommand extends Command {
  Hopper hopper;

  public ReverseIndexCommand(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setTriggerDutyCycle(TriggerConstants.triggerDutyCycleSpeedReverse.get());
    Logger.recordOutput("Commands/Reverse Index Command is Running", true);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    Logger.recordOutput("Commands/Reverse Index Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
