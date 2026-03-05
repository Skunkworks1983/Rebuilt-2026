// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * This class is a grouping of constants
 *
 * <ul>
 *   Only to be used with The talonFX collector IO for the right collector
 */
public class CollectorTalonRightConstants extends CollectorModuleConstants {

  public CollectorTalonRightConstants() { // TODO: make constants
    // Collector Side
    collectorSide = Side.RIGHT;

    // Hardware ID's
    turnMotorID = 23;
    intakeMotorID = 24;

    // Gear ratio's
    turnMotorGearRatio = 39.375;

    // Real Turn PID
    turnkP = 70;
    turnkD = 0.0;
    turnkI = 0.0;

    // Real intake PID
    intakekP = 0.0;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0.5;

    // Sim Turn PID
    SimTurnkP = 13.5;
    SimTurnkD = 12.5;
    SimTurnkI = 0.15;

    // Sim intake PID
    SimIntakekP = 0.0;
    SimIntakekD = 0.0;
    SimIntakekI = 0.0;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.12;
    turnkA = 0.01;

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorDownPos = Rotations.of(.2);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(8);

    // Limit Switch Constants
    UpperLimitChannel = 9;
    LowerLimitChannel = 7;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(3);
  }
}
