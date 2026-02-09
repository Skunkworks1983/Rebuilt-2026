// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Collector constants class - shared PID values with separate motor IDs for each side */ 
public class CollectorConstants {
  // Collector side enum
  public static enum Side {
    LEFT,
    RIGHT;
  }

  // Left Collector Hardware ID's
  public static final int LEFT_TURN_MOTOR_ID = 0;
  public static final int LEFT_INTAKE_MOTOR_LOWER_ID = 0;
  public static final int LEFT_INTAKE_MOTOR_UPPER_ID = 0;

  // Right Collector Hardware ID's
  public static final int RIGHT_TURN_MOTOR_ID = 0;
  public static final int RIGHT_INTAKE_MOTOR_LOWER_ID = 0;
  public static final int RIGHT_INTAKE_MOTOR_UPPER_ID = 0;

  // Gear ratio's
  public static final double TURN_MOTOR_GEAR_RATIO = 0.0;

  // Turn PID (shared between both collectors)
  public static final double TURN_KP = 1.0;
  public static final double TURN_KD = 0.0;
  public static final double TURN_KI = 0.0;

  // Lower intake PID (shared between both collectors)
  public static final double LOWER_KP = 0.0;
  public static final double LOWER_KD = 0.0;
  public static final double LOWER_KI = 0.0;

  // Upper intake PID (shared between both collectors)
  public static final double UPPER_KP = 0.0;
  public static final double UPPER_KD = 0.0;
  public static final double UPPER_KI = 0.0;

  // Feed forward (shared between both collectors)
  public static final double TURN_KS = 0.25;
  public static final double TURN_KV = 0.12;
  public static final double TURN_KA = 0.01;

  // Position
  public static final Angle COLLECTOR_UP_POS = Degrees.of(90);
  public static final Angle COLLECTOR_DOWN_POS = Degrees.of(0);

  // Collector Intake Speeds
  public static final AngularVelocity COLLECTOR_INTAKE_SPEED = RotationsPerSecond.of(130);
}
