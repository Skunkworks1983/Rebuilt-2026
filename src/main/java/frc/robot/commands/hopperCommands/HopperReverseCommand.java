// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopperCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import org.littletonrobotics.junction.Logger;

/** Command that runs the hopper */
public class HopperReverseCommand extends Command {
  Hopper hopper;

  public HopperReverseCommand(Hopper hopper) {
    this.hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setHopperVelocity(
        RotationsPerSecond.of(HopperConstants.hopperAngularVelocityRotationsPerSecond.get() * -1));
    Logger.recordOutput("Hopper/Reverse Command is Running", true);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput("Hopper/Reverse Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
