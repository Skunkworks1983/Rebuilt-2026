// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightCollectorDownState extends Command {
  private Collector collector;
  private Angle rightDownAngle;
  private Angle leftUpAngle;

  /** Creates a new RightCollectorDownState. */
  public RightCollectorDownState(Collector collector, Angle rightDownAngle, Angle leftUpAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);

    this.rightDownAngle = rightDownAngle;
    this.leftUpAngle = leftUpAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!collector.getRightCollector().isCollectorDown()) {
      if (!collector.getLeftCollector().isCollectorDown()) {
        collector.getRightCollector().setCollectorPos(rightDownAngle);
      } else if (collector.getLeftCollector().isCollectorDown()) {
        collector.getLeftCollector().setCollectorPos(leftUpAngle);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (collector.getRightCollector().isCollectorDown()
        && !collector.getLeftCollector().isCollectorDown()) {
      return true;
    } else {
      return false;
    }
  }
}
