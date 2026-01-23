// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Main CollectorIO, Implemented by both back and front collectors.
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

  /**
   * Gets the position of the collector
   *
   * @return
   */
  public default Angle getPosition() {
    return Angle.ofBaseUnits(0, null);
  }

  /**
   * Gets the velocity of the collectors intake upper motor
   *
   * @return
   */
  public default AngularVelocity getVelocityOfIntakeLower() {
    return AngularVelocity.ofBaseUnits(0, null);
  }

  /**
   * Gets the velocity of the collectors intake lower motor
   *
   * @return
   */
  public default AngularVelocity getVelocityOfIntakeUpper() {
    return AngularVelocity.ofBaseUnits(0, null);
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return
   */
  public default AngularVelocity getVelocityOfTurn() {
    return AngularVelocity.ofBaseUnits(0, null);
  }

  /**
   * Gets the turn motors gear ratio
   *
   * @return
   */
  public default double getTurnGearRatio() {
    return 0.0;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param
   */
  public default void setCollectorVelocity(AngularVelocity velocity) {}

  /**
   * Sets the collectors position
   *
   * @param
   */
  public default void setCollectorPos(Angle angle) {}

  /**
   * Sets the collectors turn motor velocity
   *
   * @param
   */
  public default void setCollectorTurnMotorVelocity(AngularVelocity velocity) {}

  /**
   * Checks to see if the collector is down This is to help ensure that we only have one collector
   * down at a time
   */
  public default boolean isCollectorDown() {
    return false;
  }
}
