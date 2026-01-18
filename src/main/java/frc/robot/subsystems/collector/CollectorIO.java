package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorIO {
  @AutoLog
  public static class CollectorIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRad = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(CollectorIOInputs inputs) {}

  public default void setCollectorVelocity(double velocity) {}
}
