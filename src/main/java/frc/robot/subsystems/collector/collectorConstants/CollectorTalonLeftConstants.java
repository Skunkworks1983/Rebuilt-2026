// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

/**
 * This class is a grouping of constants
 *
 * <ul>
 *   Only to be used with The talonFX collector IO for the left collector
 */
public class CollectorTalonLeftConstants extends CollectorModuleConstants {

  public CollectorTalonLeftConstants() { // TODO: make constants
    // Collector Side
    collectorSide = Side.LEFT;

    // Hardware ID's
    turnMotorID = 22;
    intakeMotorID = 21;

    // Gear ratio's
    turnMotorGearRatio = 39.375;

    // Real Turn PID
    turnkP = 80;
    turnkD = 1.0;
    turnkI = 0.0;

    // Real intake PID
    intakekP = 0;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0.5;

    // Sim Turn PID
    SimTurnkP = 13.5;
    SimTurnkD = 12.5;
    SimTurnkI = 0.4;

    // Sim intake PID
    SimIntakekP = 0.0;
    SimIntakekD = 0.0;
    SimIntakekI = 0.0;

    // feed forward
    turnkS = 0.25;
    turnkV = 0.4;
    turnkA = 0.03;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(8);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(10);

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorDownPos = Radians.of(1.65);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(35);

    // Limit Switch Constants
    UpperLimitChannel = 6;
    LowerLimitChannel = 8;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);
  }
}
