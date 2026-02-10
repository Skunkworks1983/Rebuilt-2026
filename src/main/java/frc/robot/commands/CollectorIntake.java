// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/orgit  share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kitCollector.Collector;
import frc.robot.subsystems.kitCollector.CollectorConstants;
import frc.robot.subsystems.simpleShooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorIntake extends Command {
  private Collector collector;
  LoggedTunableNumber collectorVelocity =
        new LoggedTunableNumber(
            "Collector/VelocitySetpointRotPerSec",
            CollectorConstants.defaultCollectorVelocity.in(RotationsPerSecond));

  /** Creates a new collectorIntake. */
  public CollectorIntake(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.intake(AngularVelocity.ofBaseUnits(collectorVelocity.get(), DegreesPerSecond));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.intake(AngularVelocity.ofBaseUnits(0, DegreesPerSecond));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
