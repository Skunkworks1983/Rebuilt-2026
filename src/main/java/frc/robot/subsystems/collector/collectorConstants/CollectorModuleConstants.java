// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/** An Interface for collector constants */
public class CollectorModuleConstants {
  // Collector side
  public static enum Side {
    LEFT,
    RIGHT;
  }

  // Collectors side
  public Side collectorSide;

  // Hardware ID's
  public int turnMotorID;
  public int intakeMotorID;

  // Gear ratio's
  public double turnMotorGearRatio;

  // Turn PID
  public double turnkP;
  public double turnkD;
  public double turnkI;

  // Upper intake PID
  public double upperkP;
  public double upperkD;
  public double upperkI;

  // Feed forward
  public double turnkS;
  public double turnkV;
  public double turnkA;

  public AngularVelocity pivotMaxVelocity = RotationsPerSecond.of(0);
  public AngularAcceleration pivotMaxAcceleration = RotationsPerSecondPerSecond.of(0);

  // Position
  public Angle CollectorUpPos;
  public Angle CollectorDownPos;

  // Collector Intake Speeds
  public AngularVelocity CollectorIntakeSpeed;

  // Limit Switch Constants
  public int UpperLimitChannel;
  public int LowerLimitChannel;

  // Collector Position Tolerance
  public Angle CollectorPosTolerance;

  public CollectorModuleConstants() {

    // Collector Side
    collectorSide = Side.LEFT;

    // Hardware ID's
    turnMotorID = 0;
    intakeMotorID = 0;

    // Gear ratio's
    turnMotorGearRatio = 0.0;

    // Turn PID
    turnkP = 1;
    turnkD = 0.0;
    turnkI = 0.0;

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

    // Limit Switch Constants
    UpperLimitChannel = 0;
    LowerLimitChannel = 0;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(0);
  }
}
