// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.Side;
import org.littletonrobotics.junction.AutoLog;

/**
 * Main CollectorModuleIO, Implemented by both left and right collectors.
 * <ul> Base for other collector IO's as well as sending collector data to advantage scope
 *
 * @author Liam B
 */
public interface CollectorModuleIO {
  @AutoLog
  public static class CollectorIOInputs {
    // Turn Motor Inputs
    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public AngularVelocity turnVelocity = DegreesPerSecond.of(0.0);
    public Voltage turnAppliedVolts = Volts.of(0.0);
    public Current turnCurrentAmps = Amps.of(0.0);
    public double turnClosedLoopError = 0.0;

    // Lower Intake Motor Inputs
    public boolean lowerConnected = false;
    public Angle lowerPositionRad = Degrees.of(0.0);
    public AngularVelocity lowerVelocity = DegreesPerSecond.of(0.0);
    public Voltage lowerAppliedVolts = Volts.of(0.0);
    public Current lowerCurrentAmps = Amps.of(0.0);

    // Upper Intake Motor Inputs
    public boolean upperConnected = false;
    public Angle upperPositionRad = Degrees.of(0.0);
    public AngularVelocity upperVelocity = DegreesPerSecond.of(0.0);
    public Voltage upperAppliedVolts = Volts.of(0.0);
    public Current upperCurrentAmps = Amps.of(0.0);
  }

  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  public default void updateInputs(CollectorIOInputs inputs) {}

  /** Gets Constants */
  public default CollectorConstants getConstants() {
    return null;
  }

  /**
   * Gets the turn motors gear ratio
   *
   * @return -Turn motors gear ratio
   */
  public default double getTurnGearRatio() {
    return 0.0;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public default void setCollectorVelocity(AngularVelocity velocity) {}

  /**
   * Sets the collectors position
   *
   * @param angle WPILib angle
   */
  public default void setCollectorPos(Angle angle) {}

  /**
   * Sets the collectors turn motor velocity
   *
   * @param velocity -Angular Velocity
   */
  public default void setCollectorTurnMotorVelocity(AngularVelocity velocity) {}

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public default boolean isCollectorDown() {
    return false;
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public default boolean isCollectorUp() {
    return false;
  }

  // gets side i.e: left
  public default Side getSide() {
    return null;
  }

  // Sets collector up
  public default void goUp() {}

  // Sets collector down
  public default void goDown() {}
}
