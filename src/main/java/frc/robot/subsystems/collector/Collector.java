// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the collector subsystem, uses IO's to switch between front and back
 *
 * @author Liam B
 */
public class Collector extends SubsystemBase {
    private final CollectorIO collectorIO;
    private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

    /**@param collectorIO
     */
    public Collector(CollectorIO collectorIO) {
        this.collectorIO = collectorIO;
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(inputs);
    }

    
    /**Sets the velocity of our collector motors
     * @param
    */
    public void setCollectorVelocity(AngularVelocity velocity) {
        collectorIO.setCollectorVelocity(velocity);
    }

    /**Puts the collector into a down position
     * @param
     */
    public void setCollectorDown(Angle angle) {
        collectorIO.setCollectorDown(angle);
    }

    /**Puts the collector into an up position
     * @param
     */
    public void setCollectorUp(Angle angle) {
        collectorIO.setCollectorUp(angle);
    }

    /**Checks to see if the collector is down
     * This is to help ensure that we only have one collector down at a time
     */
    public boolean canCollectorGoDown() {
        return collectorIO.canCollectorGoDown();
    }
}
