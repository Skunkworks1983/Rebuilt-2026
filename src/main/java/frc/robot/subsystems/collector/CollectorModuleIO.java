// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.Side;
import org.littletonrobotics.junction.AutoLog;

/**
 * Main CollectorModuleIO, Implemented by both left and righht collectors.
 *
 * @author Liam B
 */
public interface CollectorModuleIO {
  @AutoLog
  public static class CollectorIOInputs {
    // Turn Motor Inputs
    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public boolean turnUpperLimitSwitch = false;

    // Lower Intake Motor Inputs
    public boolean lowerConnected = false;
    public double lowerPositionRad = 0.0;
    public double lowerVelocityRadPerSec = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerCurrentAmps = 0.0;

    // Upper Intake Motor Inputs
    public boolean upperConnected = false;
    public double upperPositionRad = 0.0;
    public double upperVelocityRadPerSec = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperCurrentAmps = 0.0;
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
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public default void setCollectorVelocity(AngularVelocity velocity) {}

  /**
   * Checks to see if the collector is up
   */
  public default boolean isCollectorUp() {
    return false;
  }

  /**
   * Zeros the collector position when limit switch is triggered
   */
  public default void zeroPosition() {}

  // Sets collector up
  public default void goUp() {}

  // Sets collector down
  public default void goDown() {}
}
