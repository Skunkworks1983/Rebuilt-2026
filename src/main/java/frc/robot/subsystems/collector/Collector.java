// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.collectorCommands.AutomaticStateChange;
import frc.robot.commands.collectorCommands.BothCollectorsUpState;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;

/**
 * This is the collector Subsystem
 *
 * <ul>
 *   <li>It takes in two collector module IOs
 * </ul>
 */
public class Collector extends SubsystemBase {

  private final CollectorModuleIO leftIO;
  private final CollectorModuleIO rightIO;
  private final CollectorIOInputsAutoLogged leftInputs = new CollectorIOInputsAutoLogged();
  private final CollectorIOInputsAutoLogged rightInputs = new CollectorIOInputsAutoLogged();
  private DoubleSupplier direction;

  public static enum Mode {
    MANUAL,
    AUTOMATIC,
    ALWAYSUP;
  }

  public static enum CollectorState {
    LEFT_DOWN,
    RIGHT_DOWN,
    UP;
  }

  Mode mode = Mode.ALWAYSUP;
  CollectorState currentState = CollectorState.UP;
  CollectorState targetState = CollectorState.UP;

  /**
   * Collector subsystem.
   *
   * <ul>
   *   <li>Takes in two collector module IOs
   * </ul>
   *
   * <p><img src="https://media.giphy.com/media/jVFgWDtkY5v2M/giphy.gif" alt="Our Collector">
   *
   * @param leftIO left collector IO
   * @param rightIO right collector IO
   */
  public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO, DoubleSupplier direction) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;
    this.direction = direction;
  }

  @Override
  public void periodic() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);
    Logger.processInputs("Collector/Left", leftInputs);
    Logger.processInputs("Collector/Right", rightInputs);
  }

  /**
   * Gets the position of the left collector
   *
   * @return -WPILib Angle
   */
  public Angle getLeftPosition() {
    return leftInputs.turnPosition.getMeasure();
  }

  /**
   * Gets the position of the right collector
   *
   * @return -WPILib Angle
   */
  public Angle getRightPosition() {
    return rightInputs.turnPosition.getMeasure();
  }

  /**
   * Sets the velocity of left collector intake motors
   *
   * @param velocity -Angular Velocity
   */
  public void setLeftCollectorVelocity(AngularVelocity velocity) {
    leftIO.setCollectorVelocity(velocity);
  }

  /**
   * Sets the velocity of right collector intake motors
   *
   * @param velocity -Angular Velocity
   */
  public void setRightCollectorVelocity(AngularVelocity velocity) {
    rightIO.setCollectorVelocity(velocity);
  }

  /**
   * Checks to see if the left collector is down
   */
  public boolean isLeftCollectorDown() {
    return leftIO.isCollectorDown();
  }

  /**
   * Checks to see if the right collector is down
   */
  public boolean isRightCollectorDown() {
    return rightIO.isCollectorDown();
  }

  /**
   * Gets the constants for the left collector
   */
  public CollectorConstants getLeftConstants() {
    return leftIO.getConstants();
  }

  /**
   * Gets the constants for the right collector
   */
  public CollectorConstants getRightConstants() {
    return rightIO.getConstants();
  }

  /**
   * Sets the collector state with safe transitions.
   * If wrong collector is down, puts it up first before putting the commanded one down.
   *
   * @param state the desired collector state
   * @return Command that transitions to the desired state
   */
  public Command setCollectorState(CollectorState state) {
    targetState = state;
    
    switch (state) {
      case LEFT_DOWN:
        if (isRightCollectorDown()) {
          // Right is down, need to put it up first
          return Commands.sequence(
            Commands.runOnce(() -> rightIO.goUp(), this),
            Commands.waitUntil(() -> !isRightCollectorDown()),
            Commands.runOnce(() -> {
              leftIO.goDown();
              currentState = CollectorState.LEFT_DOWN;
            }, this)
          );
        } else {
          // Safe to put left down directly
          return Commands.runOnce(() -> {
            leftIO.goDown();
            currentState = CollectorState.LEFT_DOWN;
          }, this);
        }
        
      case RIGHT_DOWN:
        if (isLeftCollectorDown()) {
          // Left is down, need to put it up first
          return Commands.sequence(
            Commands.runOnce(() -> leftIO.goUp(), this),
            Commands.waitUntil(() -> !isLeftCollectorDown()),
            Commands.runOnce(() -> {
              rightIO.goDown();
              currentState = CollectorState.RIGHT_DOWN;
            }, this)
          );
        } else {
          // Safe to put right down directly
          return Commands.runOnce(() -> {
            rightIO.goDown();
            currentState = CollectorState.RIGHT_DOWN;
          }, this);
        }
        
      case UP:
        return Commands.runOnce(() -> {
          leftIO.goUp();
          rightIO.goUp();
          currentState = CollectorState.UP;
        }, this);
        
      default:
        return Commands.none();
    }
  }

  /**
   * Gets the current collector state
   */
  public CollectorState getCurrentState() {
    return currentState;
  }

  // Changes Mode
  public void changeMode(Mode desiredMode) {
    mode = desiredMode;
    // Collector State Control
    switch (desiredMode) {
      case ALWAYSUP:
        new BothCollectorsUpState(this).schedule();
        break;

      case MANUAL:
        break;

      case AUTOMATIC:
        new AutomaticStateChange(this, direction).schedule();
        break;

      default:
        break;
    }
  }

  // Gets Mode
  public Mode getMode() {
    return mode;
  }
}
