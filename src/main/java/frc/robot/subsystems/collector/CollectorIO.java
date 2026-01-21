// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Main CollectorIO, Implemented by both back and front collectors.
 * 
 * @author Liam B
 */
public interface CollectorIO {
    @AutoLog
    public static class CollectorIOInputs {
        // Turn Motor Inputs
        public boolean turnConnected = false;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        // Lower Intake Motor Inputs
        public boolean lowerConnected = false;
        public double lowerPositionRad = 0.0;
        public double lowerVelocityRadPerSec = 0.0;
        public double lowerAppliedVolts = 0.0;
        public double lowerCurrentAmps = 0.0;

        // Upper Intake Motor Inputs
        public boolean upperConnected = false;
        public double upperPositionRad = 0.0;
        public double upperVelocityRadPerSec = 0.0;
        public double upperAppliedVolts = 0.0;
        public double upperCurrentAmps = 0.0;
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
    public default void setCollectorPos(Angle angle) {}


    /**Checks to see if the collector is down
     * This is to help ensure that we only have one collector down at a time
     */
    public default boolean canCollectorGoDown() {
        return false;
    }
}
