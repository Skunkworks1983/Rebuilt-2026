package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  /**
   * Inputs container for the pivot hardware interface.
   *
   * <p>Implementations should populate an instance of this class inside {@link
   * #updateInputs(PivotIOInputs)} so the rest of the robot code can read sensor and state
   * information in a structured way. Fields are intentionally public for simple and fast logging.
   */
  @AutoLog
  public static class PivotIOInputs {
    /** True when the pivot hardware is connected and responding. */
    public boolean motorConnected = false;

    public boolean encoderConnected = false;

    /** The absolute encoder position of the pivot where 0 equals a launch angle of 0. */
    public Angle pivotAbsolutePosition = Rotations.of(0);

    public AngularVelocity pivotVelocity = RotationsPerSecond.of(0);

    /** The internal encoder position of the pivot where 0 equals a launch angle of 0. */
    public Angle pivotPosition = Rotations.of(0);

    /** The voltage currently applied to the pivot motor. */
    public Voltage pivotAppliedVolts = Volts.of(0);

    /** The motor stator current in amps. */
    public Current pivotStatorCurrentAmps = Amps.of(0);

    /** The current used for field oriented torque control */
    public Current pivotTorqueCurrentAmps = Amps.of(0);
  }

  /**
   * Command the pivot to a specific position
   *
   * @param angle target angle for the pivot, where 0 degrees is a launch angle of 0 degrees
   */
  public default void setPosition(Angle angle) {}

  /**
   * Apply an open-loop torque current command to the pivot motor.
   *
   * <p>This is primarily for sysid
   *
   * @param current desired current to apply to the pivot motor
   */
  public default void setOpenLoopTorqueCurrent(Current current) {}

  /**
   * Update the provided {@link PivotIOInputs} instance with the latest sensor and status readings
   * from the hardware.
   *
   * @param inputs mutable inputs object to populate with the latest hardware values
   */
  public default void updateInputs(PivotIOInputs inputs) {}

  /**
   * Reconfigures the motor on the fly to use new set of PID and FF gains
   *
   * @param gains controller gains
   */
  public default void setPID(Slot0Configs gains) {}

  /**
   * Set motion constraints for motion-profile generation on the pivot.
   *
   * @param maxVelocity maximum angular velocity allowed for the pivot
   * @param maxAcceleration maximum angular acceleration allowed for the pivot
   */
  public default void setMotionConstraints(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {}
}
