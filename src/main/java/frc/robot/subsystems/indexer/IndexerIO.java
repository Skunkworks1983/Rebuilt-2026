// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** this is the Indexers' IO */
public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public Angle position;
    public AngularVelocity angularVelocity;
    public Voltage appliedVolts;
    public Current currentAmps;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** sets velocity at which to run the indexer */
  public default void setIndexerVelocity(AngularVelocity velocity) {}

  /** sets duty cycle at which to run the indexer */
  public default void setIndexerDutyCycle(double dutyCycle) {}
}
