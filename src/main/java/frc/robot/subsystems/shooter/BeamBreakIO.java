// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** The current beam break is most likely a stand in for now until a real one can be made */
public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean connected = false;

    public boolean beamBroken = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}
}
