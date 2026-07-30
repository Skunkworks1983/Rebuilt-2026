// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveAimedAutoShoot;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import org.littletonrobotics.junction.Logger;

/**
 * Command that runs the trigger Note: This ONLY runs as the trigger, acting as an override, and
 * does not spin up the flywheel
 */
public class ForceRunTrigger extends Command {
  Hopper hopper;

  DriveAimedAutoShoot driveAimedShoot;

  public ForceRunTrigger(Hopper hopper, DriveAimedAutoShoot driveAimedShoot) {
    this.hopper = hopper;
    this.driveAimedShoot = driveAimedShoot;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    hopper.triggerRunMaxDutyCycle();
    hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
    driveAimedShoot.lockControls();

    Logger.recordOutput("Trigger/Run Trigger Command is Running", true);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput("Trigger/Run Trigger Command is Running", false);

    driveAimedShoot.unlockControls();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
