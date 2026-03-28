// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class is a grouping of constants
 *
 * <ul>
 *   Only to be used with The talonFX collector IO for the right collector /** This class is a
 *   grouping of constants for the collector
 */
public class CollectorModuleConstants {
  // Collector CanBus
  public CANBus canbus;

  // Hardware ID's
  public int turnMotorID;
  public int intakeMotorID;

  // Gear ratio's
  public double turnMotorGearRatio;

  // Real Turn PID
  public double turnkP;
  public double turnkD;
  public double turnkI;

  // Real turn invert
  public InvertedValue turnInvert;

  // Real intake PID
  public double intakekP;
  public double intakekD;
  public double intakekI;
  public double intakekV;
  public double intakekS;
  public double intakekA;

  // Sim Turn PID
  public double SimTurnkP;
  public double SimTurnkD;
  public double SimTurnkI;

  // Sim intake PID
  public double SimIntakekP;
  public double SimIntakekD;
  public double SimIntakekI;

  // Feed forward
  public double turnkS;
  public double turnkV;
  public double turnkA;

  // Motion Magic constraints
  public AngularVelocity motionMagicCruiseVelocity;
  public AngularAcceleration motionMagicAcceleration;

  // Position
  public Angle CollectorUpPos;
  public LoggedTunableNumber CollectorDownPos;
  public LoggedTunableNumber CollectorHalfPos;

  // Collector Manual pivot velocity
  public double CollectorPivotDutyCycle;

  // Collector Intake Speeds
  public AngularVelocity CollectorIntakeSpeed;

  // Limit Switch Constants
  public int UpperLimitChannel;
  public int LowerLimitChannel;

  // Collector Position Tolerance
  public Angle CollectorPosTolerance;

  // Collector SupplyCurrents
  public double turnSupplyCurrent;
  public double intakeSupplyCurrent;

  // Inverted Value
  public InvertedValue intakeInvertedValue;

  public CollectorModuleConstants() {

    // Hardware IDs
    turnMotorID = 32;
    intakeMotorID = 21;

    // Gear ratio's
    turnMotorGearRatio = 29.07160494;

    // Real Turn PID
    turnkP = 20;
    turnkD = 0;
    turnkI = 0;

    // Real intake upper PID
    intakekP = 15;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = 0;
    intakekA = 0;
    intakekS = 0;
    intakeInvertedValue = InvertedValue.CounterClockwise_Positive;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(0.5);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(2);

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorDownPos = new LoggedTunableNumber("Collector/Downpose rad", 2.198);
    CollectorHalfPos = new LoggedTunableNumber("Collector/Halfpose rad", 1);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RadiansPerSecond.of(80);

    // Limit Switch Constants
    UpperLimitChannel = 1;
    LowerLimitChannel = 2;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);
  }
}
