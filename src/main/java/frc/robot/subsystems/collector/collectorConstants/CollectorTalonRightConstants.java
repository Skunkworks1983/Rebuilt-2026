// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.util.LoggedTunableNumber;

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
    turnkP = 80;
    turnkD = 1.0;
    turnkI = 0;

    // Real Turn PID
    turnkPSlot1 = 8;
    turnkDSlot1 = .1;
    turnkISlot1 = 0.0;

    // Inverted value
    turnInvert = InvertedValue.Clockwise_Positive;

    // Real intake PID
    intakekP = 4.5;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0;
    intakekA = 9;
    intakekS = 8;

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
    turnkV = 0.4;
    turnkA = 0.03;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(7);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(9);

    // Position
    CollectorUpPos = Rotations.of(-0.02);

    CollectorDownPos = new LoggedTunableNumber("Collector/Right/Downpose rad", 1.55);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RadiansPerSecond.of(80);

    // Limit Switch Constants
    UpperLimitChannel = 9;
    LowerLimitChannel = 7;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);
  }
}
