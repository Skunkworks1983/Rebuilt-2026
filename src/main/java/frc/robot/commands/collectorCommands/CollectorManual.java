// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorManual extends Command {
  private Collector collector;
  private Joystick buttons;
  private CollectorConstants constants;
  private String downCollector;
  private String upCollector;

  /** Creates a new CollectorManual. */
  public CollectorManual(Collector collector, Joystick buttons, CollectorConstants constants) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    this.buttons = buttons;
    this.constants = constants;
    addRequirements(collector);

    downCollector = null;
    upCollector = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    downCollector = collector.getDownCollector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (downCollector.equals("LEFT")) {
      upCollector = "RIGHT";
    } else {
      upCollector = "LEFT";
    }

    
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
