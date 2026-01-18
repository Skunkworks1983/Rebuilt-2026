package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public Voltage shooterAppliedVolts = Volts.of(0);
    public Current shooterCurrentAmps = Amps.of(0);
  }

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setVoltage(double voltage) {}

  public default void updateInputs(FlywheelIOInputs inputs) {}
}
