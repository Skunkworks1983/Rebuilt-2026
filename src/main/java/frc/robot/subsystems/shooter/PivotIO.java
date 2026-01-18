package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean connected = false;
    public Rotation2d pivotAbsolutePosition = Rotation2d.kZero;
    public Rotation2d pivotPosition = Rotation2d.kZero;
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrentAmps = Amps.of(0);
  }

  public default void setPosition(Rotation2d angle) {}

  public default void setVoltage(double voltage) {}

  public default void updateInputs(PivotIOInputs inputs) {}
}
