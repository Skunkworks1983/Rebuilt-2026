// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants.Side;
import org.littletonrobotics.junction.Logger;

/**
 * This is the collector module subsystem
 *
 * <ul>
 *   <li>Uses IO's to switch between left and right<\li>
 *       <ul>
 *
 * @author Liam B
 */
public class CollectorModule {
  private final CollectorModuleIO collectorIO;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  /**
   * This is the collector module subsystem
   *
   * <ul>
   *   <li>Uses IO's to switch between left and right
   * </ul>
   *
   * @param collectorIO
   */
  public CollectorModule(CollectorModuleIO collectorIO) {
    this.collectorIO = collectorIO;
  }

  public void periodic() {
    collectorIO.updateInputs(inputs);
    Logger.processInputs("Collector/Module/ " + collectorIO.getSide(), inputs);
  }

  /** Gets Constants */
  public CollectorModuleConstants getConstants() {
    return collectorIO.getConstants();
  }

  /**
   * Gets the position of the collector
   *
   * @return -WPILib Angle
   */
  public Angle getPosition() {
    return inputs.turnPosition.getMeasure();
  }

  /**
   * Gets the velocity of the collectors intake motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfIntake() {
    return inputs.intakeVelocity;
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfTurn() {
    return inputs.turnVelocity;
  }

  /**
   * Sets the velocity of our collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    collectorIO.setCollectorVelocity(velocity);
  }

  /**
   * Puts the collector into a down position
   *
   * @param angle WPILib angle
   */
  public void setCollectorPos(Angle angle) {
    collectorIO.setCollectorPos(angle);
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorDown() {
    return collectorIO.isCollectorDown();
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorUp() {
    return collectorIO.isCollectorUp();
  }

  // Sets collector up
  public void goUp() {
    collectorIO.goUp();
  }

  // Sets collector down
  public void goDown() {
    collectorIO.goDown();
  }

  public Side getSide() {
    return collectorIO.getSide();
  }
}
