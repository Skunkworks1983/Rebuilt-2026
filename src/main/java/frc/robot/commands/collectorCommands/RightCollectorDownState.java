// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightCollectorDownState extends Command {
  private Collector collector;
  private int count;

  /**
   * Creates a new RightCollectorDownState.
   *
   * <h2>Implementation </h2>
   *
   * <ol>
   *   <li>Ensures Left Collector is up
   *   <li>Sends Right Collector down
   * </ol>
   *
   * @param collector The collector subsystem
   */
  public RightCollectorDownState(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    Logger.recordOutput("Commands/Collector/RightCollectorDownState", true);
    if (collector.getLeftCollector().isCollectorUp()) {
      collector.getLeftCollector().setCollectorEncoderPos(Rotations.of(0));
    }
    if (collector.getRightCollector().isCollectorUp()) {
      collector.getRightCollector().setCollectorEncoderPos(Rotations.of(0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collector.getLeftCollector().isCollectorUp()) {
      collector.getRightCollector().goDown();
    } else {
      collector.getLeftCollector().goUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Collector/RightCollectorDownState", false);
    if (collector.getLeftCollector().isCollectorUp()) {
      collector.getLeftCollector().setCollectorEncoderPos(Rotations.of(0));
    }
    if (collector.getRightCollector().isCollectorUp()) {
      collector.getRightCollector().setCollectorEncoderPos(Rotations.of(0));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(collector.getRightCollector().isCollectorDown()
        && collector.getLeftCollector().isCollectorUp()) {
          count++;
    } else {
      count = 0;
    }
    return count >= 3;
  }
}
