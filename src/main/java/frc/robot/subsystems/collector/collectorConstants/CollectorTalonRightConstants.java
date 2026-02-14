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
 *   Only to be used with The talonFX collector IO for the right collector
 */
public class CollectorTalonRightConstants extends CollectorConstants {

  public CollectorTalonRightConstants() {
    // Collector Side
    collectorSide = Side.RIGHT;

    // Hardware ID's
    turnMotorID = 1;
    intakeMotorLowerID = 0;
    intakeMotorUpperID = 0;

    // Gear ratio's
    turnMotorGearRatio = 0.0;

    // Turn PID
    turnkP = 5;
    turnkD = 0;
    turnkI = 0.0;

    // Lower intake PID
    lowerkP = 0.0;
    lowerkD = 0.0;
    lowerkI = 0.0;

    // Upper intake PID
    upperkP = 0.0;
    upperkD = 0.0;
    upperkI = 0.0;

    // Feed forward
    turnkS = 0.25;
    turnkV = 0.12;
    turnkA = 0.01;

    // Position
    CollectorUpPos = Degrees.of(0);
    CollectorDownPos = Degrees.of(90);

    // Collector Intake Speeds
    CollectorIntakeSpeed = RotationsPerSecond.of(0);
  }
}
