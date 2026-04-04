// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.triggerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.BedConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHopperToJoystick extends Command {
  Hopper hopper;
  Shooter shooter;
  DoubleSupplier hopperSpeed;
  Supplier<AngularVelocity> triggerSpeed;
  /** Creates a new setHopperToJoystick. */
  public SetHopperToJoystick(
      Hopper hopper,
      DoubleSupplier hopperSpeed,
      Supplier<AngularVelocity> triggerSpeed,
      Shooter shooter) {
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper, shooter);
    this.hopperSpeed = hopperSpeed;
    this.triggerSpeed = triggerSpeed;
    addRequirements(hopper, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setManualVelocity(RotationsPerSecond.of(30));
    hopper.setHopperDutyCycle(BedConstants.hopperDutyCycleSpeedFeed.get());
    hopper.setTriggerVelocity(triggerSpeed.get());
    Logger.recordOutput("Hopper/SetHopperToJoystick Command", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setHopperDutyCycle(0);
    hopper.setTriggerDutyCycle(0);
    Logger.recordOutput("Hopper/SetHopperToJoystick Command", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
