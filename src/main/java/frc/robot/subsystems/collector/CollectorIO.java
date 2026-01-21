// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.units.measure.AngularVelocity;

/** Main CollectorIO, Implemented by both back and front collectors.
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

    /**Updates our inputs for all motors 
     * @param
    */
    public default void updateInputs(CollectorIOInputs inputs) {}

    /**Sets the velocity of our collector motors
     * @param
    */
    public default void setCollectorVelocity(AngularVelocity velocity) {}

    /**Puts the collector into a down position
     * @param
     */
    public default void setCollectorDown(AngularVelocity velocity) {}

    /**Puts the collector into an up position
     * @param
     */
    public default void setCollectorUp(AngularVelocity velocity) {}

    /**Checks to see if the collector is down
     * This is to help ensure that we only have one collector down at a time
     */
    public default boolean canCollectorGoDown() {
        return false;
    }
}
