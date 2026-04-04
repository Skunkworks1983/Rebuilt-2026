// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** this is the Triggers' IO */
public interface TriggerIO {
  @AutoLog
  public static class TriggerIOInputs {
    public boolean primaryConnected = false;
    public Angle primaryPosition;
    public AngularVelocity primaryAngularVelocity;
    public Voltage primaryAppliedVolts;
    public Current primaryCurrentAmps;
    public double primaryClosedLoopError;

    public boolean followerConnected = false;
    public Angle followerPosition;
    public AngularVelocity followerAngularVelocity;
    public Voltage followerAppliedVolts;
    public Current followerCurrentAmps;
    public double followerClosedLoopError;
  }

  public default void updateInputs(TriggerIOInputs inputs) {}

  /** sets velocity at which to run the trigger */
  public default void setTriggerVelocity(AngularVelocity velocity) {}

  /**
   * Reconfigure PID/FF gains used by the flywheel controller on the fly.
   *
   * @param configs controller gains
   */
  public default void setPID(Slot0Configs configs) {}

  /**
   * Apply an open-loop torque/current command to the flywheel motor.
   *
   * <p>This is primarily for SysID
   *
   * @see <a href="https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/9">
   *     SysID for FOC Torque Control</a>
   */
  public default void setOpenLoopTorqueCurrent(Current output) {}

  public default void setDutyCycle(double dutyCycle) {}

  public default void setVoltageOut(Voltage voltage) {}
}
