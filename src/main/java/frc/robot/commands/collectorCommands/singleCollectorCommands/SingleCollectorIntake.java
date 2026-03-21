// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands.singleCollectorCommands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.singleCollector.SingleCollector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SingleCollectorIntake extends Command {
  SingleCollector collector;
  /** Creates a new singleCollectorIntake. */
  public SingleCollectorIntake(SingleCollector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/SingleCollectorIntake", true);
    collector.getSingleCollector().setCollectorPos(
            Radians.of(collector.getSingleCollector().getConstants().CollectorDownPos.get()), 0);
    collector
        .getSingleCollector()
        .setCollectorVelocity(collector.getSingleCollector().getConstants().CollectorIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Collector/SingleCollectorIntake", false);
    collector
        .getSingleCollector()
        .setCollectorPos(
            Radians.of(collector.getSingleCollector().getConstants().CollectorHalfPos.get()), 0);
    collector.getSingleCollector().setCollectorDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
