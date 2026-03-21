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

    // Hardware ID's
    turnMotorID = 32;
    intakeMotorID = 21;

    // Gear ratio's
    turnMotorGearRatio = 29.07160494;

    // Real Turn PID
    turnkP = 15;
    turnkD = 0;
    turnkI = 0;

    // Real Turn PID
    turnkPSlot1 = 8;
    turnkDSlot1 = .1;
    turnkISlot1 = 0.0;

    // Real intake upper PID
    intakekP = 15;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0;
    intakekA = 0;
    intakekS = 0;
    intakeInvertedValue = InvertedValue.CounterClockwise_Positive;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.4;
    turnkA = 0.03;

    // Feed forward slot 1
    turnkSSlot1 = 0;
    turnkVSlot1 = 0;
    turnkASlot1 = 0;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(0.5);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(2);

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorDownPos = new LoggedTunableNumber("Collector/Single/Downpose rad", 2.198);
    CollectorHalfPos = new LoggedTunableNumber("Collector/Single/Halfpose rad", 1);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RadiansPerSecond.of(80);

    // Limit Switch Constants
    UpperLimitChannel = 1;
    LowerLimitChannel = 2;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);
  }
}
