// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

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

/** this is the Hoppers' IO */
public interface BedIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.of(0);
    public AngularVelocity angularVelocity = RotationsPerSecond.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current currentAmps = Amps.of(0);
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  /** sets velocity at which to run the hopper */
  public default void setHopperVelocity(AngularVelocity velocity) {}

  /**
   * Reconfigure PID/FF gains used by the hopper controller on the fly.
   *
   * @param configs controller gains
   */
  public default void setPID(Slot0Configs configs) {}

  /**
   * Apply an open-loop torque/current command to the hopper motor.
   *
   * <p>This is primarily for SysID
   *
   * @see <a href="https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/9">
   *     SysID for FOC Torque Control</a>
   */
  public default void setOpenLoopTorqueCurrent(Current output) {}

  public default void setDutyCycle(double dutyCycle) {}
}
