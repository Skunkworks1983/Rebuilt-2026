// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorManual extends Command {
  /** Creates a new CollectorManual. */
  public CollectorManual(Joystick j, Collector c) {
    // Use addRequirements() here to declare subsystem dependencies.
    joystick = j;
    collector = c;
  }

  Collector collector;
  Joystick joystick;
  CollectorModule lCollector;
  CollectorModule rCollector;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lCollector = collector.getLeftCollector();
    rCollector = collector.getRightCollector();

    rCollector.setCollectorVelocity(AngularVelocity.ofBaseUnits(0, null));
    lCollector.setCollectorVelocity(AngularVelocity.ofBaseUnits(0, null));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rCollector.setCollectorVelocity(AngularVelocity.ofBaseUnits(joystick.getY(), null));
    lCollector.setCollectorVelocity(AngularVelocity.ofBaseUnits(joystick.getX(), null));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
