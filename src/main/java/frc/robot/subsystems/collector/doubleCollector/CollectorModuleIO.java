// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.doubleCollector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants.Side;
import org.littletonrobotics.junction.AutoLog;

/**
 * Main CollectorModuleIO, Implemented by both left and right collectors.
 *
 * <ul>
 *   Base for other collector IO's as well as sending collector data to advantage scope
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
    public double turnSetPoint = 0.0;

    // Intake Motor Inputs for lower or double collector
    public boolean intakeConnected = false;
    public Angle intakePositionRad = Degrees.of(0.0);
    public AngularVelocity intakeVelocity = DegreesPerSecond.of(0.0);
    public Voltage intakeAppliedVolts = Volts.of(0.0);
    public Current intakeCurrentAmps = Amps.of(0.0);

    // Magnet Switch Inputs
    public boolean upperTriggered = false;
    public boolean lowerTriggered = false;

    // When were on our way down
    public boolean goingDown = false;

    // up and down
    public boolean isUp = true;
    public boolean isDown = false;
  }

  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  public default void updateInputs(CollectorIOInputs inputs) {}

  /** Gets Constants */
  public default CollectorModuleConstants getConstants() {
    return new CollectorModuleConstants();
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public default void setCollectorIntakeVelocity(AngularVelocity velocity) {}

  /** sets Just the encoder pos */
  public default void setCollectorEncoderPos(Angle angle) {}

  /**
   * Sets the collectors position
   *
   * @param angle WPILib angle
   */
  public default void setCollectorPos(Angle angle, int slot) {}

  /**
   * Sets the velocity of our pivot motor collector motors
   *
   * @param velocity -Angular Velocity
   */
  public default void setCollectorPivotDutyCycle(double dutyCycle) {}

  /**
   * Sets the duty cycle of the intake motors
   *
   * @param dutyCycle double between -1.0 and 1.0
   */
  public default void setCollectorIntakeDutyCycle(double dutyCycle) {}

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
   * Checks to see if the collector is up
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

  public default void setPivotGains(Slot0Configs slotConfigs) {}

  public default void setIntakeGains(Slot0Configs slot0Configs) {}

  public default void setMotionConstraints(MotionMagicConfigs motionMagicConfigs) {}
  ;
}
