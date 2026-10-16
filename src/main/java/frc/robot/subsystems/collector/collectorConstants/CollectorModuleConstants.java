// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;

/** This class is a grouping of constants */
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
  public static final Angle CollectorDownPosDefault = Radians.of(2.198);
  public Angle CollectorHalfPosDefault;

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
  public double turnStatorCurrent;
  public double intakeSupplyCurrent;

  // Inverted Value
  public InvertedValue intakeInvertedValue;

  // Agitate period
  public double agitatePeriod;

  public CollectorModuleConstants() {

    // Hardware IDs
    turnMotorID = 24;
    intakeMotorID = 23;

    canbus = new CANBus("peripherals_2026");

    // Gear ratio's
    turnMotorGearRatio = (28.0 / 12.0) * (58.0 / 18.0) * (58.0 / 18.0) * (20.0 / 12);
    turnInvert = InvertedValue.Clockwise_Positive;
    turnSupplyCurrent = 15;

    // Real Turn PID
    turnkP = 45;
    turnkD = 0;
    turnkI = 0;

    // Real intake upper PID
    intakekP = 5;
    intakekD = 0.0;
    intakekI = 0.0;
    intakekV = (5.0 / 80.0) * 3.5;
    intakekA = 0;
    intakekS = 0;
    intakeInvertedValue = InvertedValue.CounterClockwise_Positive;
    intakeSupplyCurrent = 80;
    turnStatorCurrent = 80;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(1.3);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(3.5);

    // Position
    CollectorUpPos = Rotations.of(0);
    CollectorHalfPosDefault = Radians.of(1);
    CollectorDownPos =
        new LoggedTunableNumber("Collector/Downpose rad", CollectorDownPosDefault.in(Radians));
    CollectorHalfPos =
        new LoggedTunableNumber("Collector/Halfpose rad", CollectorHalfPosDefault.in(Radians));

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(105 * 1.25);

    // Limit Switch Constants
    UpperLimitChannel = 1;
    LowerLimitChannel = 2;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(2);

    // Agitate period
    agitatePeriod = .75;
  }
}
