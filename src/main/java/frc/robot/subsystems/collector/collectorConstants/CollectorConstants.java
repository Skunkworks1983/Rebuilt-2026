// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** An Interface for collector constants */
public interface CollectorConstants {
  // Collector side
  public static enum side {
    LEFT,
    RIGHT;
  }

  public static side collectorSide =
      frc.robot.subsystems.collector.collectorConstants.CollectorConstants.side.LEFT;

  // Hardware ID's
  public static final int turnMotorID = 0;
  public static final int intakeMotorLowerID = 0;
  public static final int intakeMotorUpperID = 0;

  // Gear ratio's
  public static final double turnMotorGearRatio = 0.0;

  // Turn PID
  public static final double turnkP = 0.0;
  public static final double turnkD = 0.0;
  public static final double turnkI = 0.0;

  // Lower intake PID
  public static final double lowerkP = 0.0;
  public static final double lowerkD = 0.0;
  public static final double lowerkI = 0.0;

  // Upper intake PID
  public static final double upperkP = 0.0;
  public static final double upperkD = 0.0;
  public static final double upperkI = 0.0;

  // Feed forward
  public static final double turnkS = 0.25;
  public static final double turnkV = 0.12;
  public static final double turnkA = 0.01;

  // Position
  public static final Angle CollectorUpPos = Angle.ofBaseUnits(0, Degrees);
  public static final Angle CollectorDownPos = Angle.ofBaseUnits(0, Degrees);

  // Collector Intake Speeds
  public static final AngularVelocity CollectorIntakeSpeed = RotationsPerSecond.of(0);
}
