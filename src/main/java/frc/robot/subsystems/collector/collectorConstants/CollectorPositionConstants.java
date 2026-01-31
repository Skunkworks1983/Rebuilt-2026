// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/** These are the collector positions, ONLY POSITIONS */
public class CollectorPositionConstants {
    // Collector Up positions
    public final static Angle leftCollectorUpPos = Angle.ofBaseUnits(0.0, Degrees);
    public final static Angle rightCollectorUpPos = Angle.ofBaseUnits(0.0, Degrees);
    
    // Collector Down Positions
    public final static Angle leftCollectorDownPos = Angle.ofBaseUnits(0.0, Degrees);
    public final static Angle rightCollectorDownPos = Angle.ofBaseUnits(0.0, Degrees);
}
