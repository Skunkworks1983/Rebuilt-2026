// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightCollectorDownState extends Command {
  private Collector collector;

  /** Creates a new RightCollectorDownState. */
  public RightCollectorDownState(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/RightCollectorDownState", true);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collector.getRightCollector().isCollectorDown()
        && collector.getLeftCollector().isCollectorUp();
  }
}
