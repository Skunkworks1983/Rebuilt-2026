// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopperCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import org.littletonrobotics.junction.Logger;

/** Command that runs the hopper */
public class HopperVelocityCommand extends Command {
  Hopper hopper;
  AngularVelocity velocity;

  public HopperVelocityCommand(Hopper hopper, AngularVelocity velocity) {
    this.hopper = hopper;
    this.velocity = velocity;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.setHopperVelocity(velocity);
    Logger.recordOutput("Hoppper/Command is Running", true);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput("Hopper/Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
