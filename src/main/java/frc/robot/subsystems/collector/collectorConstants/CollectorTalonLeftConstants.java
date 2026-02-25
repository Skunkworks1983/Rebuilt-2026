// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * This class is a grouping of constants with get methods
 *
 * <ul>
 *   Only to be used with The talonFX collector IO for the left collector
 */
public class CollectorTalonLeftConstants extends CollectorModuleConstants {

  public CollectorTalonLeftConstants() { // TODO: make constants
    // Collector Side
    collectorSide = Side.LEFT;

    // Hardware ID's
    turnMotorID = 59;
    intakeMotorID = 50;

    // Gear ratio's
    turnMotorGearRatio = 0.0;

    // Real Turn PID
    turnkP = 0.0;
    turnkD = 0.0;
    turnkI = 0.0;

    // Real intake PID
    intakekP = 0.0;
    intakekD = 0.0;
    intakekI = 0.0;

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
    turnkV = 0.12;
    turnkA = 0.01;

    // Position
    CollectorUpPos = Degrees.of(0);
    CollectorDownPos = Degrees.of(90);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(0);

    // Limit Switch Constants
    UpperLimitChannel = 24;
    LowerLimitChannel = 23;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(10);
  }
}
