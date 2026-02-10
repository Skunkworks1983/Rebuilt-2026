// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitIndexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public AngularVelocity indexerVelocity = RotationsPerSecond.of(0);
    public Voltage indexerVoltage = Volts.of(0);
    public Current indexerCurrent = Amps.of(0);
    public Current indexerTorqueCurrent = Amps.of(0);
    public Angle indexerPosition = Rotations.of(0);
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void intake(AngularVelocity velocity) {}

  public default void setPID(Slot0Configs gains) {}
}
