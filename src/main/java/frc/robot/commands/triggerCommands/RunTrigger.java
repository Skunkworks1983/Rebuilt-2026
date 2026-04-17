// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/** Command that runs the trigger */
public class RunTrigger extends Command {
  Hopper hopper;
  Shooter shooter;

  public RunTrigger(Hopper hopper, Shooter shooter, Drive drive) {
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Trigger/Run Trigger Command is Running", true);
    hopper.setTriggerDutyCycle(-1.0);
    hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput("Trigger/Run Trigger Command is Running", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
