// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModule;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorExpel extends Command {
  Collector collector;
  CollectorModule leftCollector;
  CollectorModule rightCollector;
  CollectorModule lastModule;

  /** Creates a new CollectorReverseIntake. */
  public CollectorExpel(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    leftCollector = collector.getLeftCollector();
    rightCollector = collector.getRightCollector();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/CollectorExpel", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (leftCollector.isCollectorDown() && lastModule != leftCollector) {
      rightCollector.setCollectorDutyCycle(0);
      leftCollector.setCollectorVelocity(
          leftCollector.getConstants().CollectorIntakeSpeed.unaryMinus());
      lastModule = leftCollector;
    } else if (rightCollector.isCollectorDown() && lastModule != rightCollector) {
      leftCollector.setCollectorDutyCycle(0);
      rightCollector.setCollectorVelocity(
          rightCollector.getConstants().CollectorIntakeSpeed.unaryMinus());
      lastModule = rightCollector;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftCollector.setCollectorDutyCycle(0);
    rightCollector.setCollectorDutyCycle(0);
    Logger.recordOutput("Commands/Collector/CollectorExpel", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
