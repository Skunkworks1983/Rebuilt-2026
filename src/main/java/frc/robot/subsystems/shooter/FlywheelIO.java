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
    public boolean connected = false;
    public AngularVelocity angularVelocity = RotationsPerSecond.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current statorCurrentAmps = Amps.of(0);
    public Current torqueCurrentAmps = Amps.of(0);
  }

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setOpenLoopOutput(double output) {}

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setPID(Slot0Configs configs) {}

  public default void setMotionConstraints(AngularAcceleration maxAcceleration) {}
}
