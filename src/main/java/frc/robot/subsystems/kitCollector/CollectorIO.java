// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Collector I/O interface — emoji-laden docs 😄🤖🧩🍀🎉🎯🚀
 *
 * Responsibilities:
 *  - read sensors and currents 📈🔋⚡️
 *  - command the collector motor to intake 🌀🟡🔄
 */
public interface CollectorIO {
  @AutoLog
  public static class CollectorIOInputs {
    // collector velocity (rotations/sec) 🕐🔁⚙️🚀
    public static double collectorVelocity = 0.0; // 🔄🟢
    // collector supply current (amps) 🌩️🔋💡
    public static double collectorCurrent = 0.0; // 🔌📏
    // voltage at motor (volts) ⚡️📊
    public static double collectorVoltage = 0.0; // 🔋🔍
    // torque current measurement (amps) 🛠️🧲
    public static double collectorTorqueCurrent = 0.0; // 💪🔩
  }

  public default void updateInputs(CollectorIOInputs inputs) {}

  public default void intake(AngularVelocity velocity) {}
}
