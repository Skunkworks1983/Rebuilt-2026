// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
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

  // Real intake PID
  public double intakekP;
  public double intakekD;
  public double intakekI;
  public double intakekV;

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
  public Angle CollectorDownPos;

  // Collector Manual pivot velocity
  public double CollectorPivotVelocity;

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

    // Collector canBus
    canbus = new CANBus("peripherals_2026");

    // Hardware ID's
    turnMotorID = 0;
    intakeMotorID = 0;

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
    intakekV = 0.0;

    // Sim Turn PID
    SimTurnkP = 0.0;
    SimTurnkD = 0.0;
    SimTurnkI = 0.0;

    // Sim intake PID
    SimIntakekP = 0.0;
    SimIntakekD = 0.0;
    SimIntakekI = 0.0;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.12;
    turnkA = 0.01;

    // Motion Magic constraints
    motionMagicCruiseVelocity = RotationsPerSecond.of(1.7);
    motionMagicAcceleration = RotationsPerSecondPerSecond.of(14);

    // Position
    CollectorUpPos = Degrees.of(0);
    CollectorDownPos = Degrees.of(0);

    // Pivot Velocity
    CollectorPivotVelocity = 0.01;

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(0);

    // Limit Switch Constants
    UpperLimitChannel = 0;
    LowerLimitChannel = 0;

    // Collector Position Tolerance
    CollectorPosTolerance = Degrees.of(0);
  }
}
