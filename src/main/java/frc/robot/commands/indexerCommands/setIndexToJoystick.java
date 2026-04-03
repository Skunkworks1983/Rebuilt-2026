// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexerCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIndexToJoystick extends Command {
  Hopper hopper;
  Shooter shooter;
  DoubleSupplier hopperSpeed;
  DoubleSupplier indexerSpeed;
  /** Creates a new setIndexToJoystick. */
  public setIndexToJoystick(
      Hopper hopper, DoubleSupplier hopperSpeed, DoubleSupplier indexerSpeed, Shooter shooter) {
    this.hopper = hopper;
    this.shooter = shooter;
    addRequirements(hopper, shooter);
    this.hopperSpeed = hopperSpeed;
    this.indexerSpeed = indexerSpeed;
    addRequirements(hopper, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setManualVelocity(RotationsPerSecond.of(30));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.setHopperVelocity(RotationsPerSecond.of(hopperSpeed.getAsDouble()));
    hopper.setIndexerVelocity(RotationsPerSecond.of(indexerSpeed.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setHopperDutyCycle(0);
    hopper.setIndexerDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
