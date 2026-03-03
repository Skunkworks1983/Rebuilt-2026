// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collectorCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModule;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.previouseSwitch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectorConfig extends Command {
  private Collector collector;
  private CollectorModule leftCollector;
  private CollectorModule rightCollector;
  private CollectorModule lastModule;
  private boolean automaticSwitch;
  private boolean bothUpSwitch;
  private boolean manualSwitch;
  private BooleanSupplier intake;
  private BooleanSupplier shooting;
  private BooleanSupplier manualAgitation;
  private BooleanSupplier leftDownManual;
  private BooleanSupplier rightDownManual;
  private previouseSwitch previousModeSwitch;
  private previouseSwitch currSwitch;
  private Supplier<LinearVelocity> direction; //(+/-)
  private double currDirection;
  private Debouncer debouncer;
  private boolean isGoingRight;
  private boolean wasGoingRight;

  /** Creates a new CollectorConfig. 
   *  <p> This Command intakes buttons and the collector subsystem to decide how to collectors should act with eachother 
   *  and Move, Intake, and Agitate the collector pivot and intake motors.
   * 
   *  <p>
   *  <h1> Implementation </h1>
   * 
   *  <ol>
   *    <li> Checks which mode its in based on a 3-position switch
   *    <li> Runs a different initialize and execute based on said switch
   *    <ul>
   *      <li> <strong> Automatic </strong>
   *        <ol>
   *          <li> Uses a debounced current direction {@link LinearVelocity} to decide (using +/-) what direction its going
   *          <li> Using whether it is going right and whether it WAS going right it changes which collector it wants up or down
   *          <li> Sets the collector it wants up, up, and then waits for it to be FULLY up before sending the collector it wants down, down
   *          <li> ON END, both collector will go UP
   *        </ol>
   *      <li> <strong> Both Up </strong>
   *        <ol>
   *          <li> Sends both collectors up, whether they already are or not (Though if it is up it wont move)
   *        </ol>
   *      <li> <strong> Manual </strong>
   *        <ol>
   *          <li> Gets which button has been pressed (leftDown or rightDown)
   *          <li> Pulls the opposite collector up
   *          <li> Sends Respective Collector Down when other collector is up
   *        </ol>
   *    </ul>
   *    <li> Also facilitates interactions between button presses
   *      <ul>
   *        <li> When moving from auto mode to manual mode dont go up
   *        <li> When moving out of auto mode but intaking dont go up
   *        <li> Otherwise raise up
   *      </ul>
   *  </ol>
   * 
   *  @param collector - The collector subsystem, contains both sides
   *  @param automaticSwitch - Automatic switch button (part of 3-position switch)
   *  @param bothUpSwitch - Both Up switch button (part of 3-position switch), true if auto and manual switches are false
   *  @param manualSwitch - Manual switch button (part of 3-position switch)
   *  @param intake - Intake button, controls whether were intaking or not
   *  @param shooting - Shooting button, tells us whether were shooting so we can decide if we should agitate or not
   *  @param manualAgitation - Agitation button, tells us to agitate in manual
   *  @param leftDownManual - Tells us to put the right collector up and left down in manual
   *  @param rightDownManual - Tells us to put the left collector up and right down in manual
   *  @param direction - A {@link LinearVelocity} that gives us a direction (- == right, + == left)
  */
  public CollectorConfig(Collector collector, boolean automaticSwitch, boolean bothUpSwitch, boolean manualSwitch, 
      BooleanSupplier intake, BooleanSupplier shooting, BooleanSupplier manualAgitation, 
      BooleanSupplier leftDownManual, BooleanSupplier rightDownManual, Supplier<LinearVelocity> direction) {
    this.collector = collector;
    addRequirements(collector);

    // Switches
    this.automaticSwitch = automaticSwitch;
    this.bothUpSwitch = bothUpSwitch;
    this.manualSwitch = manualSwitch;
    previousModeSwitch = CollectorConstants.preSwitch;
    this.intake = intake;
    this.shooting = shooting;
    this.manualAgitation = manualAgitation;
    this.leftDownManual = leftDownManual;
    this.rightDownManual = rightDownManual;
    this.direction = direction;

    if(automaticSwitch) {
      currSwitch = previousModeSwitch.automatic;
    }
    if(bothUpSwitch) {
      currSwitch = previousModeSwitch.bothUp;
    }
    if(manualSwitch) {
      currSwitch = previousModeSwitch.manual;
    }

    // Mode variable
    debouncer = new Debouncer(0.06, DebounceType.kBoth);
    isGoingRight = false;
    wasGoingRight = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Collector/CollectorConfig", true);
    // Stop collectors from going up when command starts if were leaving auto and going to manual or if intaking and leaving auto
    if((previousModeSwitch.equals(previouseSwitch.automatic) && currSwitch.equals(previouseSwitch.manual)) 
      || (intake.getAsBoolean() && previousModeSwitch.equals(previouseSwitch.automatic) 
      && currSwitch.equals(previouseSwitch.automatic))) {
      // IDK what happens here ig
    } else {
      leftCollector.goUp();
      rightCollector.goUp();
    }

    if(automaticSwitch) {
      currDirection = direction.get().in(MetersPerSecond);
      Logger.recordOutput("Commands/Collector/States/AutomaticStateChange", true);
    }
    if(bothUpSwitch) {
      collector.getLeftCollector().goUp();
      collector.getRightCollector().goUp();
      Logger.recordOutput("Commands/Collector/States/BothCollectorsUpState", true);
    }
    if(manualSwitch) {
      Logger.recordOutput("Commands/Collector/States/CollectorManualState", true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Automatic Mode (Based on +/- of direction)
    if(currSwitch.equals(previouseSwitch.automatic)) {
      if(intake.getAsBoolean()) { // Automatic collector should only work when holding intake

        // Automatic Position switch logic
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

        // Intake and agitate logic
        if (leftCollector.isCollectorDown() && (lastModule != leftCollector)) {
          if(shooting.getAsBoolean()) {
            rightCollector.setCollectorVelocity(rightCollector.getConstants().CollectorAgitateSpeed);
          }
          leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorIntakeSpeed);
          lastModule = leftCollector;
        } else if (rightCollector.isCollectorDown() && (lastModule != rightCollector)) {
          if(shooting.getAsBoolean()) {
            leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
          }
          rightCollector.setCollectorVelocity(rightCollector.getConstants().CollectorIntakeSpeed);
          lastModule = rightCollector;
        }

        wasGoingRight = isGoingRight;
      } else {
        rightCollector.goUp();
        leftCollector.goUp();
      }
    }

    // Both Collectors Go Up And Stay Up
    if(currSwitch.equals(previouseSwitch.bothUp)) {

      // Agitation
      if(shooting.getAsBoolean()) {
        leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
        rightCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
      }
    }

    // Manual Mode
    if(currSwitch.equals(previouseSwitch.manual)) {
      // Switch side
      if(rightDownManual.getAsBoolean()) {
        if (leftCollector.isCollectorUp()) {
          rightCollector.goDown();
        } else {
          leftCollector.goUp();
        }
      } else if(leftDownManual.getAsBoolean()) {
        if (rightCollector.isCollectorUp()) {
          leftCollector.goDown();
        } else {
          rightCollector.goUp();
        }
      }

      // Agitate
      if(manualAgitation.getAsBoolean()) {
        if(leftCollector.isCollectorUp()) {
          leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
        } else if(rightCollector.isCollectorUp()) {
          rightCollector.setCollectorVelocity(leftCollector.getConstants().CollectorAgitateSpeed);
        }
      }

      // Intake 
      if(intake.getAsBoolean()) {
        if(leftCollector.isCollectorDown()) {
          leftCollector.setCollectorVelocity(leftCollector.getConstants().CollectorIntakeSpeed);
        } else if(rightCollector.isCollectorDown()) {
          rightCollector.setCollectorVelocity(leftCollector.getConstants().CollectorIntakeSpeed);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CollectorConstants.preSwitch = currSwitch;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
