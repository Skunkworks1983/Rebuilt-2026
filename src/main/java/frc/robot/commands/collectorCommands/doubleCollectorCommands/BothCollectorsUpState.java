// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands.doubleCollectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.doubleCollector.DoubleCollector;
import org.littletonrobotics.junction.Logger;

/** The Both Collectors Up state */
public class BothCollectorsUpState extends Command {
  private DoubleCollector collector;

  /**
   * Creates a new BothCollectorUpState.
   *
   * <h2>Implementation </h2>
   *
   * <ol>
   *   <li>Sends both collectors up, whether they already are or not (Though if it is up it wont
   *       move)
   * </ol>
   *
   * @param collector The collector subsystem
   */
  public BothCollectorsUpState(DoubleCollector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.getLeftCollector().goUp();
    collector.getRightCollector().goUp();
    Logger.recordOutput("Commands/Collector/BothCollectorsUpState", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/Collector/BothCollectorsUpState", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collector.getLeftCollector().isCollectorUp()
        && collector.getRightCollector().isCollectorUp();
  }
}
