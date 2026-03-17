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
public class CollectorTalonSingleConstants extends CollectorModuleConstants {

  public CollectorTalonSingleConstants() { // TODO: do constants
    // Collector Side
    collectorSide = Side.SINGLE;

    // Hardware ID's
    turnMotorID = 0;
    intakeMotorID = 0;

    // Gear ratio's
    turnMotorGearRatio = 0;

    // Real Turn PID
    turnkP = 0;
    turnkD = 0;
    turnkI = 0;

    // Real Turn PID
    turnkPSlot1 = 8;
    turnkDSlot1 = .1;
    turnkISlot1 = 0.0;

    // Real intake upper PID
    intakekP = 0;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0;
    intakekA = 0;
    intakekS = 0;
    intakeInvertedValue = InvertedValue.Clockwise_Positive;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.4;
    turnkA = 0.03;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(0);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(0);

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorDownPos = new LoggedTunableNumber("Collector/Right/Downpose rad", 1.55);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RadiansPerSecond.of(0);

    // Limit Switch Constants
    UpperLimitChannel = 0;
    LowerLimitChannel = 0;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);
  }
}
