package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean connected = false;

    public Angle pivotAbsolutePosition = Angle.ofBaseUnits(0, Rotations);

    public Angle pivotPosition = Angle.ofBaseUnits(0, Rotations);

    public Voltage pivotAppliedVolts = Volts.of(0);

    public Current pivotStatorCurrentAmps = Amps.of(0);

    public Current pivotTorqueCurrentAmps = Amps.of(0);
  }

  public default void setPosition(Angle angle) {}

  public default void setOpenLoopCurrent(Current current) {}

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPID(Slot0Configs gains) {}

  public default void setMotionConstraints(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {}
}
