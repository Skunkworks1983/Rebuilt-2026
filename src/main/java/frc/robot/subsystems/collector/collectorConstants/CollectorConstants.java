// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

/** Add your docs here. */
public interface CollectorConstants {

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

    // Get methods for all constants
    public default int getTurnMotorID() {
        return turnMotorID;
    }

    public default int getIntakeMotorLowerID() {
        return intakeMotorLowerID;
    }

    public default int getIntakeMotorUpperID() {
        return intakeMotorUpperID;
    }

    public default double getTurnMotorGearRatio() {
        return turnMotorGearRatio;
    }

    public default double getTurnKp() {
        return turnkP;
    }

    public default double getTurnKd() {
        return turnkD;
    }

    public default double getTurnKi() {
        return turnkI;
    }

    public default double getLowerKp() {
        return lowerkP;
    }

    public default double getLowerKd() {
        return lowerkD;
    }

    public default double getLowerKi() {
        return lowerkI;
    }

    public default double getUpperKp() {
        return upperkP;
    }

    public default double getUpperKd() {
        return turnkD;
    }

    public default double getUpperKi() {
        return turnkI;
    }

    public default double getTurnKs() {
        return turnkS;
    }

    public default double getTurnKv() {
        return turnkV;
    }

    public default double getTurnKa() {
        return turnkA;
    }

}
