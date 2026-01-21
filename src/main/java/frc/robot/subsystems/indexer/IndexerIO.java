// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double positionDeg = 0.0;
    public double angularVelocityDeg = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  // specify a velocity at which to run the indexer (just a .set for now, will correct later)
  public default void setIndexerVelocity(Double VelocityDegPerSec) {}
}
