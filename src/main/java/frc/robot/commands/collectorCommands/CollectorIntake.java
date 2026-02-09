// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorIntake extends Command {
  Collector collector;

  /** Creates a new CollectorIntake. */
  public CollectorIntake(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collector.isLeftCollectorDown()) {
      collector.setLeftCollectorVelocity(collector.getLeftConstants().CollectorIntakeSpeed);
    } else if (collector.isRightCollectorDown()) {
      collector.setRightCollectorVelocity(collector.getRightConstants().CollectorIntakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setLeftCollectorVelocity(AngularVelocity.ofBaseUnits(0, DegreesPerSecond));
    collector.setRightCollectorVelocity(AngularVelocity.ofBaseUnits(0, DegreesPerSecond));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
