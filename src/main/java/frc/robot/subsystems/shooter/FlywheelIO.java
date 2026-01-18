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
    public boolean primaryConnected = false;
    public boolean followerConnected = false;

    public AngularVelocity primaryVelocity = RotationsPerSecond.of(0);
    public AngularVelocity followerVelocity = RotationsPerSecond.of(0);

    public Voltage primaryAppliedVolts = Volts.of(0);
    public Voltage followerAppliedVolts = Volts.of(0);

    public Current primaryStatorCurrentAmps = Amps.of(0);
    public Current followerStatorCurrentAmps = Amps.of(0);

    public Current primaryTorqueCurrentAmps = Amps.of(0);
    public Current followerTorqueCurrentAmps = Amps.of(0);
  }

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setOpenLoopCurrent(Current current) {}

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void setMotionConstraints(double maxAcceleration, double maxJerk) {}
}
