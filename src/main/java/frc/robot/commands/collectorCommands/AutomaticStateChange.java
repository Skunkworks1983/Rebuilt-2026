// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.Collector.CollectorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomaticStateChange extends Command {
  private Collector collector;
  private double lastDirection;
  private double currDirection;
  private DoubleSupplier direction;
  private Debouncer directionDebouncer;

  /** Creates a new AutomaticStateChange. */
  public AutomaticStateChange(Collector collector, DoubleSupplier direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    addRequirements(collector);
    this.direction = direction;
    // Debounce for 0.06 seconds (3 cycles at 50Hz) before accepting direction change
    this.directionDebouncer = new Debouncer(0.06, Debouncer.DebounceType.kRising);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currDirection = direction.getAsDouble();
    lastDirection = currDirection;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastDirection = currDirection;
    currDirection = direction.getAsDouble();
    
    // Check if direction has changed (different sign)
    boolean directionChanged = (currDirection * lastDirection) < 0;
    
    // Use debouncer to ensure direction change is sustained
    if (directionDebouncer.calculate(directionChanged)) {
      if (currDirection > 0.0) { // if going right
        collector.setCollectorState(CollectorState.RIGHT_DOWN).schedule();
      } else if (currDirection < 0.0) { // if going left
        collector.setCollectorState(CollectorState.LEFT_DOWN).schedule();
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
