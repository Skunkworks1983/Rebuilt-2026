// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the actual collector subsystem
 *
 * @author Liam B
 */
public class Collector extends SubsystemBase {
    private final CollectorIO collectorIO;
    private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

    public Collector(CollectorIO collectorIO) {
        this.collectorIO = collectorIO;
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(inputs);
    }

    public void setCollectorVelocity(AngularVelocity velocity) {
        collectorIO.setCollectorVelocity(velocity);
    }

    public void setCollectorDown(AngularVelocity velocity) {
        collectorIO.setCollectorDown(velocity);
    }

    public void setCollectorUp(AngularVelocity velocity) {
        collectorIO.setCollectorUp(velocity);
    }

    public boolean canCollectorGoDown() {
        return collectorIO.canCollectorGoDown();
    }
}
