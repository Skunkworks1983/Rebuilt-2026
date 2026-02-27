// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    /** True when the beam break hardware is connected and responding. */
    public boolean connected = false;

    /** True when the beam break is currently interrupted. */
    public boolean beamBroken = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}
}
