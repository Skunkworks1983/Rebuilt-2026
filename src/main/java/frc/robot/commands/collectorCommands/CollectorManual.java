// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@Deprecated
public class CollectorManual extends Command {
  private Collector collector;
  private Joystick buttons;
  private CollectorConstants constants;
  private String toGoDown;
  private JoystickButton switchButton;
  private JoystickButton leftDownButton;
  private JoystickButton rightDownButton;

  /** Creates a new CollectorManual. */
  public CollectorManual(Collector collector, Joystick buttons, CollectorConstants constants) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    this.buttons = buttons;
    this.constants = constants;
    addRequirements(collector);
    toGoDown = "NEITHER";
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(toGoDown.equals("LEFT")) {
      if(collector.getDownCollector().equals("RIGHT")) {
        collector.getRightCollector().setCollectorPos(null); // TODO: change this null to an actual number (up pos)
        if(collector.getRightCollector().getPosition().in(Degrees) >= 0.0) { // TODO: change 0.0 to up pos
          collector.setDownCollector("NEITHER");
        }
      } else if(collector.getDownCollector().equals("LEFT")){
        collector.getLeftCollector().setCollectorVelocity(null); // TODO: change null to collector intake velocity
      } else if(collector.getDownCollector().equals("NEITHER")) {
        collector.getLeftCollector().setCollectorPos(null); // TODO: change this null to an actual number (down pos)
        if(collector.getLeftCollector().getPosition().in(Degrees) <= 0.0) { // TODO: change this 0.0 to up pos
          collector.setDownCollector("LEFT");
        }
      }
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
