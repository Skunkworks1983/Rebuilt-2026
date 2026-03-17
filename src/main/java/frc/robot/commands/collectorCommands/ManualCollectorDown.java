// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.doubleCollector.DoubleCollector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCollectorDown extends Command {
  DoubleCollector collector;
  /** Creates a new ManualCollectorDown. */
  public ManualCollectorDown(DoubleCollector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!collector.getLeftCollector().isCollectorUp()) {
      collector
          .getLeftCollector()
          .setCollectorPivotDutyCycle(
              collector.getLeftCollector().getConstants().CollectorPivotDutyCycle);
      collector
          .getRightCollector()
          .setCollectorPos(collector.getRightCollector().getConstants().CollectorUpPos, 0);
    }
    if (!collector.getRightCollector().isCollectorUp()) {
      collector
          .getRightCollector()
          .setCollectorPivotDutyCycle(
              collector.getRightCollector().getConstants().CollectorPivotDutyCycle);
      collector
          .getLeftCollector()
          .setCollectorPos(collector.getLeftCollector().getConstants().CollectorUpPos, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.getLeftCollector().setCollectorPivotDutyCycle(0);
    collector.getRightCollector().setCollectorPivotDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
