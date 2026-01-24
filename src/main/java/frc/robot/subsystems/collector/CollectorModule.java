// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the collector subsystem, uses IO's to switch between front and back
 *
 * @author Liam B
 */
public class CollectorModule extends SubsystemBase {
  private final CollectorModuleIO collectorIO;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  /**
   * @param collectorIO
   */
  public CollectorModule(CollectorModuleIO collectorIO) {
    this.collectorIO = collectorIO;
  }

  @Override
  public void periodic() {
    collectorIO.updateInputs(inputs);
  }

  /**
   * Gets the position of the collector
   *
   * @return
   */
  public Angle getPosition() {
    return collectorIO.getPosition();
  }

  /**
   * Gets the velocity of the collectors intake upper motor
   *
   * @return
   */
  public AngularVelocity getVelocityOfIntakeLower() {
    return collectorIO.getVelocityOfIntakeLower();
  }

  /**
   * Gets the velocity of the collectors intake lower motor
   *
   * @return
   */
  public AngularVelocity getVelocityOfIntakeUpper() {
    return collectorIO.getVelocityOfIntakeUpper();
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return
   */
  public AngularVelocity getVelocityOfTurn() {
    return collectorIO.getVelocityOfTurn();
  }

  /**
   * Gets the turn motors gear ratio
   *
   * @return
   */
  public double getTurnGearRatio() {
    return collectorIO.getTurnGearRatio();
  }

  /**
   * Sets the velocity of our collector motors
   *
   * @param
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    collectorIO.setCollectorVelocity(velocity);
  }

  /**
   * Puts the collector into a down position
   *
   * @param
   */
  public void setCollectorPos(Angle angle) {
    collectorIO.setCollectorPos(angle);
  }

  /**
   * Sets the collectors turn motor velocity
   *
   * @param
   */
  public void setCollectorTurnMotorVelocity(AngularVelocity velocity) {
    collectorIO.setCollectorTurnMotorVelocity(velocity);
  }

  /**
   * sets the collector isDown. This is to help ensure that we only have one collector down at a
   * time
   */
  public void setIsCollectorDown(boolean downed) {
    collectorIO.setIsCollectorDown(downed);
  }

  /**
   * Checks to see if the collector is down This is to help ensure that we only have one collector
   * down at a time
   */
  public boolean isCollectorDown() {
    return collectorIO.isCollectorDown();
  }
}
