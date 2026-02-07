// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomaticStateChange extends Command {
  private Collector collector;
  private double lastDirection;
  private double currDirection;
  private int count;
  private DoubleSupplier direction;

  /** Creates a new AutomaticStateChange. */
  public AutomaticStateChange(Collector collector, DoubleSupplier direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currDirection = direction.getAsDouble();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastDirection = currDirection;
    currDirection = direction.getAsDouble();
    if ((currDirection * lastDirection) < 0) {
      count++;
      if (count >= 3) {
        if (currDirection > 0.0) { // if going right
          if (!collector.getLeftCollector().isCollectorDown()) {
            collector.getRightCollector().goDown();
          } else {
            collector.getLeftCollector().goUp();
          }
        } else if (currDirection < 0.0) { // if going left
          if (!collector.getRightCollector().isCollectorDown()) {
            collector.getLeftCollector().goDown();
          } else {
            collector.getRightCollector().goUp();
          }
        }
      }
    } else if ((currDirection * lastDirection) > 0) {
      count = 0;
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
