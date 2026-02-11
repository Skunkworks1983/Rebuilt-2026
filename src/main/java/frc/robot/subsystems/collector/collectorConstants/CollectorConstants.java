// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** An Interface for collector constants */
public class CollectorConstants {
  // Collector side
  public static enum Side {
    LEFT,
    RIGHT;
  }

  // Collectors Sim Position on Robot
  public static final Translation2d collectorLeftOriginTranslation =
      new Translation2d(Inches.of(13), Inches.of(2));
  public static final Translation2d collectorRightOriginTranslation =
      new Translation2d(Inches.of(-13), Inches.of(2));

  // Collectors side
  public static Side collectorSide;

  // Hardware ID's
  public final int turnMotorID;
  public final int intakeMotorLowerID;
  public final int intakeMotorUpperID;

  // Gear ratio's
  public final double turnMotorGearRatio;

  // Turn PID
  public final double turnkP;
  public final double turnkD;
  public final double turnkI;

  // Lower intake PID
  public final double lowerkP;
  public final double lowerkD;
  public final double lowerkI;

  // Upper intake PID
  public final double upperkP;
  public final double upperkD;
  public final double upperkI;

  // Feed forward
  public final double turnkS;
  public final double turnkV;
  public final double turnkA;

  // Position
  public final Angle CollectorUpPos;
  public final Angle CollectorDownPos;

  // Collector Intake Speeds
  public final AngularVelocity CollectorIntakeSpeed;

  public CollectorConstants() {

    // Collector Side
    collectorSide = Side.LEFT;

    // Hardware ID's
    turnMotorID = 0;
    intakeMotorLowerID = 0;
    intakeMotorUpperID = 0;

    // Gear ratio's
    turnMotorGearRatio = 0.0;

    // Turn PID
    turnkP = 0.0;
    turnkD = 0.0;
    turnkI = 0.0;

    // Lower intake PID
    lowerkP = 0.0;
    lowerkD = 0.0;
    lowerkI = 0.0;

    // Upper intake PID
    upperkP = 0.0;
    upperkD = 0.0;
    upperkI = 0.0;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.12;
    turnkA = 0.01;

    // Position
    CollectorUpPos = Degrees.of(0);
    CollectorDownPos = Degrees.of(0);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(0);
  }
}
