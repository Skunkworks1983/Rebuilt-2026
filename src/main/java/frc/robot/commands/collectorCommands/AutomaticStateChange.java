// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomaticStateChange extends Command {
  private Collector collector;
  private double currDirection;
  private Supplier<LinearVelocity> direction;
  private Debouncer debouncer;
  private boolean isGoingRight;
  private boolean wasGoingRight;

  /** Creates a new AutomaticStateChange. */
  public AutomaticStateChange(Collector collector, Supplier<LinearVelocity> direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    this.direction = direction;
    debouncer = new Debouncer(0.06, DebounceType.kBoth);
    isGoingRight = false;
    wasGoingRight = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currDirection = direction.get().in(MetersPerSecond);
    Logger.recordOutput("Commands/Collector/AutomaticStateChange", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currDirection = direction.get().in(MetersPerSecond);
    Logger.recordOutput("TEST/AutoCollector/currDirection", currDirection);
    isGoingRight = debouncer.calculate(currDirection < 0); // Might need to swap < to >
    Logger.recordOutput("TEST/AutoCollector/isGoingRight", isGoingRight);
    if(isGoingRight && !wasGoingRight) {
      if(collector.getRightCollector().isCollectorUp()) {
        collector.getLeftCollector().goDown();
      } else {
        collector.getRightCollector().goUp();
      }
    } else if(!isGoingRight && wasGoingRight) {
      if(collector.getLeftCollector().isCollectorUp()) {
        collector.getRightCollector().goDown();
      } else {
        collector.getLeftCollector().goUp();
      }
    }
    wasGoingRight = isGoingRight;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.getLeftCollector().goUp();
    collector.getRightCollector().goUp();
    Logger.recordOutput("Commands/Collector/AutomaticStateChange", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
