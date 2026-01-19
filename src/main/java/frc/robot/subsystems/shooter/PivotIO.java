package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean primaryConnected = false;
    public boolean followerConnected = false;

    public Angle pivotAbsolutePosition = Angle.ofBaseUnits(0, Rotations);

    public Angle pivotPrimaryPosition = Angle.ofBaseUnits(0, Rotations);
    public Angle pivotFollowerPosition = Angle.ofBaseUnits(0, Rotations);

    public Voltage pivotAppliedVolts = Volts.of(0);
    public Voltage pivotFollowerAppliedVolts = Volts.of(0);

    public Current pivotStatorCurrentAmps = Amps.of(0);
    public Current pivotFollowerStatorCurrentAmps = Amps.of(0);

    public Current pivotTorqueCurrentAmps = Amps.of(0);
    public Current pivotFollowerTorqueCurrentAmps = Amps.of(0);
  }

  public default void setPosition(Angle angle) {
  }

  public default void setOpenLoopCurrent(Current current) {
  }

  public default void updateInputs(PivotIOInputs inputs) {
  }

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
  }

  public default void setMotionConstraints(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration, double maxJerk) {
  }
}
