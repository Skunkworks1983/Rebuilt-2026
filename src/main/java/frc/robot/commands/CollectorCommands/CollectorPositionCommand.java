// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCommands;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorModule;

/**  */
public class CollectorPositionCommand extends Command {
  CollectorModule collector;
  Angle setAngle;
  double tolerance = 0.2; //revolutions

  /** Creates a new CollectorPositionCommand. */
  public CollectorPositionCommand(CollectorModule collector, Angle setAngle) {
    this.collector = collector;
    this.setAngle = setAngle;

    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.setCollectorTurnMotorVelocity(AngularVelocity.ofBaseUnits(5, RevolutionsPerSecond));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return (Math.abs(collector.getPosition().in(Revolutions) - setAngle.in(Revolutions)))
        < tolerance;
  }
}
