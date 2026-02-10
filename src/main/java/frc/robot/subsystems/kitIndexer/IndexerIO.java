// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitIndexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public static double indexerVelocity = 0.0;
    public static double indexerCurrent = 0.0;
    public static double indexerVoltage = 0.0;
    public static double indexerTorqueCurrent = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void intake(double velocity) {}
}
