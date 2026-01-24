// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorUpPos extends Command {
  private Collector collector;
  private String downCollector;
  private Angle upAngle;

  /** Creates a new CollectorUpPos. */
  public CollectorUpPos(Collector collector, Angle upAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    downCollector = null;
    this.upAngle = upAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    downCollector = collector.getDownCollector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (downCollector) {
      case "LEFT":
        collector.getLeftCollector().setCollectorPos(null);
        break;

      case "RIGHT":
        collector.getRightCollector().setCollectorPos(null);
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (collector.getLeftCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)
        || collector.getRightCollector().getPosition().in(Degrees) >= upAngle.in(Degrees)) {
      return true;
    } else {
      return false;
    }
  }
}
