// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomaticStateChange extends Command {
  private Collector collector;
  private double lastDirection;
  private double currDirection;
  private int count;
  private Supplier<LinearVelocity> direction;

  /** Creates a new AutomaticStateChange. */
  public AutomaticStateChange(Collector collector, Supplier<LinearVelocity> direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currDirection = direction.get().in(MetersPerSecond);
    count = 0; // Count is to give a small wait period before changing which collector is down
    Logger.recordOutput("Commands/Collector/AutomaticStateChange", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currDirection = direction.get().in(MetersPerSecond);
    if ((currDirection * lastDirection) < 0) {
      count++;
      lastDirection = currDirection;
      if (count >= CollectorConstants.collectorTicksForAuto) {
        if (currDirection > 0.0) { // if going right
          if (collector.getLeftCollector().isCollectorUp()) {
            collector.getRightCollector().goDown();
          } else {
            collector.getLeftCollector().goUp();
          }
        } else if (currDirection < 0.0) { // if going left
          if (collector.getRightCollector().isCollectorUp()) {
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
  public void end(boolean interrupted) {
    if (collector.getLeftCollector().isCollectorDown()) {
      collector.getLeftCollector().goUp();
    }
    if (collector.getRightCollector().isCollectorDown()) {
      collector.getRightCollector().goUp();
    }
    Logger.recordOutput("Commands/Collector/AutomaticStateChange", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
