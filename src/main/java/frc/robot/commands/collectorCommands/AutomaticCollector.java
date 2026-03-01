// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModule;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The Automatic State for the collector
*/
public class AutomaticCollector extends Command {
  private Collector collector;
  private CollectorModule leftCollector;
  private CollectorModule rightCollector;
  private CollectorModule lastModule;
  private double currDirection;
  private Supplier<LinearVelocity> direction;
  private Debouncer debouncer;
  private boolean isGoingRight;
  private boolean wasGoingRight;
  private boolean shouldAgitate;
  private boolean shouldStow;

  /**Creates a new AutomaticStateChange.
   * <h2> Implementation </h2>
   * 
   * <ol>
   *  <li> Uses a debounced current direction {@link LinearVelocity} to decide (using +/-) what direction its going
   *  <li> Using whether it is going right and whether it WAS going right it changes which collector it wants up or down
   *  <li> Sets the collector it wants up, up, and then waits for it to be FULLY up before sending the collector it wants down, down
   *  <li> ON END, both collector will go UP
   * </ol>
   * 
   * @param collector The collector subsystem
   * @param direction The {@link LinearVelocity} of the robot (+ = left, - = right)
   * @param shouldAgitate Whether we should be agitating
   * @param shouldStow Whether we should stow on end
   */
  public AutomaticCollector(Collector collector, Supplier<LinearVelocity> direction, boolean shouldAgitate, boolean shouldStow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collector = collector;
    leftCollector = collector.getLeftCollector();
    rightCollector = collector.getRightCollector();
    addRequirements(collector);
    this.direction = direction;
    debouncer = new Debouncer(0.06, DebounceType.kBoth);
    isGoingRight = false;
    wasGoingRight = true;
    this.shouldAgitate = shouldAgitate;
    this.shouldStow = shouldStow;
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
    isGoingRight = debouncer.calculate(currDirection < 0);
    Logger.recordOutput("TEST/AutoCollector/isGoingRight", isGoingRight);
    if(isGoingRight && !wasGoingRight) {
      if(rightCollector.isCollectorUp()) {
        leftCollector.goDown();
      } else {
        rightCollector.goUp();
      }
    } else if(!isGoingRight && wasGoingRight) {
      if(leftCollector.isCollectorUp()) {
        rightCollector.goDown();
      } else {
        leftCollector.goUp();
      }
    }
    
    if (leftCollector.isCollectorDown() && lastModule != leftCollector) {
      if(shouldAgitate) {
        rightCollector.setCollectorVelocity(rightCollector.getConstants().CollectorAgitateSpeed);
      }
      leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorIntakeSpeed);
      lastModule = leftCollector;
    } else if (rightCollector.isCollectorDown() && lastModule != rightCollector) {
      if(shouldAgitate) {
        leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
      }
      rightCollector.setCollectorVelocity(rightCollector.getConstants().CollectorIntakeSpeed);
      lastModule = rightCollector;
    }

    wasGoingRight = isGoingRight;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(shouldStow) {
      leftCollector.goUp();
      rightCollector.goUp();
    }
    leftCollector.setCollectorVelocity(RotationsPerSecond.of(0));
    rightCollector.setCollectorVelocity(RotationsPerSecond.of(0));
    Logger.recordOutput("Commands/Collector/AutomaticStateChange", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
