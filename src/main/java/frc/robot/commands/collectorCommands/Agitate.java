// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Agitate extends Command {
  Collector collector;
  CollectorModule leftCollector;
  CollectorModule rightCollector;
  CollectorModule lastModule;
  boolean shouldIntake;
  /** Creates a new Agitate. */
  public Agitate(Collector collector, boolean shouldIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    leftCollector = collector.getLeftCollector();
    rightCollector = collector.getRightCollector();
    this.shouldIntake = shouldIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shouldIntake) {
      if (leftCollector.isCollectorDown() && lastModule != leftCollector) {
        rightCollector.setCollectorVelocity(rightCollector.getConstants().CollectorAgitateSpeed);
        lastModule = leftCollector;
      } else if (rightCollector.isCollectorDown() && lastModule != rightCollector) {
        leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
        lastModule = rightCollector;
      }
    } else { // Makes Assumption that if not intaking both are up
      leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
      rightCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftCollector.setCollectorVelocity(RotationsPerSecond.of(0));
    rightCollector.setCollectorVelocity(RotationsPerSecond.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
