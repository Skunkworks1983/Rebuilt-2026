// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.collectorConstants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class CollectorConstants {

  // X, Y pos from robot center
  public static double inchesFromOriginX = 13;
  public static double inchesFromOriginY = 2;

  // Collectors Sim Position on Robot
  public static final Translation2d collectorLeftOriginTranslation =
      new Translation2d(Inches.of(inchesFromOriginX), Inches.of(inchesFromOriginY));
  public static final Translation2d collectorRightOriginTranslation =
      new Translation2d(Inches.of(-inchesFromOriginX), Inches.of(inchesFromOriginY));

  // Collector tick number for auto state change
  public static final double collectorTicksForAuto = 0.0;
}
