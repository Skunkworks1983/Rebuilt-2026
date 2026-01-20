// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

/**
 * This is the IO for the collector
 *
 * @author Liam B
 */
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

    public default void setCollectorDown(double velocity) {}

    public default void setCollectorUp(double velocity) {}

    public default boolean canCollectorGoDown() {
        return false;
    }
}
