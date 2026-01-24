// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

/** Add your docs here. */
public class CollectorTalonRightConstants implements CollectorConstants {
  // Hardware ID's
  public static final int turnMotorID = 0;
  public static final int intakeMotorLowerID = 0;
  public static final int intakeMotorUpperID = 0;

  // Gear ratio's
  public static final double turnMotorGearRatio = 0.0;

  // Turn PID
  public static final double turnkP = 0.0;
  public static final double turnkD = 0.0;
  public static final double turnkI = 0.0;

  // Lower intake PID
  public static final double lowerkP = 0.0;
  public static final double lowerkD = 0.0;
  public static final double lowerkI = 0.0;

  // Upper intake PID
  public static final double upperkP = 0.0;
  public static final double upperkD = 0.0;
  public static final double upperkI = 0.0;

  // Feed forward
  public static final double turnkS = 0.25;
  public static final double turnkV = 0.12;
  public static final double turnkA = 0.01;

  @Override
  // Get methods for all constants
  public int getTurnMotorID() {
    return turnMotorID;
  }

  public int intakeMotorLowerID() {
    return intakeMotorLowerID;
  }

  public int intakeMotorUpperID() {
    return intakeMotorUpperID;
  }

  public double getTurnMotorGearRatio() {
    return turnMotorGearRatio;
  }

  public double getTurnKp() {
    return turnkP;
  }

  public double getTurnKd() {
    return turnkD;
  }

  public double getTurnKi() {
    return turnkI;
  }

  public double getLowerKp() {
    return lowerkP;
  }

  public double getLowerKd() {
    return lowerkD;
  }

  public double getLowerKi() {
    return lowerkI;
  }

  public double getUpperKp() {
    return upperkP;
  }

  public double getUpperKd() {
    return turnkD;
  }

  public double getUpperKi() {
    return turnkI;
  }

  public double getTurnKs() {
    return turnkS;
  }

  public double getTurnKv() {
    return turnkV;
  }

  public double getTurnKa() {
    return turnkA;
  }
}
