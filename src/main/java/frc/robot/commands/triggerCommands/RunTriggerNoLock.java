// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/** Command that runs the trigger */
public class RunTriggerNoLock extends Command {
  Hopper hopper;
  Shooter shooter;

  private boolean feeding = false;

  public RunTriggerNoLock(Hopper hopper, Shooter shooter) {
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    // hopper.setTriggerDutyCycle(
    // shooter.getFlywheelSetpoint().times(TriggerConstants.triggerVelocityMultiplier.get()));

    Logger.recordOutput("Trigger/Run Trigger Command is Running", true);
    feeding = false;
  }

  @Override
  public void execute() {
    if (!feeding && shooter.canShoot()) {
      hopper.setTriggerDutyCycle(-1.0);
      hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
      feeding = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setTriggerDutyCycle(0);
    hopper.setHopperDutyCycle(0);
    Logger.recordOutput("Trigger/Run Trigger Command is Running", false);

    feeding = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
