package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    /** True when the flywheel hardware is connected and responding. */
    public boolean connected = false;

    /** Current measured angular velocity of the flywheel */
    public AngularVelocity angularVelocity = RotationsPerSecond.of(0);

    /** Voltage currently applied to the flywheel motor. */
    public Voltage appliedVolts = Volts.of(0);

    /** Motor stator current in amps. */
    public Current statorCurrentAmps = Amps.of(0);

    /** Torque oriented control applied current */
    public Current torqueCurrentAmps = Amps.of(0);
  }

  /**
   * Set a closed-loop velocity setpoint for the flywheel.
   *
   * @param velocity desired flywheel angular velocity
   */
  public default void setVelocity(AngularVelocity velocity) {}

  /**
   * Apply an open-loop torque/current command to the flywheel motor.
   *
   * <p>This is primarily for SysID
   */
  public default void setOpenLoopTorqueCurrent(Current output) {}

  /**
   * Update the {@link FlywheelIOInputs} instance with the latest sensor and status readings from
   * the hardware.
   *
   * <p>Implementations should fill all fields of the supplied inputs object.
   *
   * @param inputs mutable inputs object to populate with the latest hardware values
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Reconfigure PID/FF gains used by the flywheel controller on the fly.
   *
   * @param configs controller gains
   */
  public default void setPID(Slot0Configs configs) {}

  /**
   * Set motion constraints for the flywheel's motion profiling or closed-loop control.
   *
   * @deprecated Motion magic has been removed from the flywheel
   * @param maxAcceleration maximum angular acceleration allowed for the flywheel
   */
  public default void setMotionConstraints(AngularAcceleration maxAcceleration) {}
}
