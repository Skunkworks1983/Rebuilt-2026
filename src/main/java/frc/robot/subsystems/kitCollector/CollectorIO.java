// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CollectorIO {
  @AutoLog
  public static class CollectorIOInputs {
    public static double collectorVelocity = 0.0;
    public static double collectorCurrent = 0.0;
    public static double collectorVoltage = 0.0;
    public static double collectorTorqueCurrent = 0.0;
  }

  public default void updateInputs(CollectorIOInputs inputs) {}

  public default void intake(AngularVelocity velocity) {}
}
