// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Collector extends SubsystemBase{

    private final CollectorIO io;
    private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();
    public Collector(CollectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic () {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public void intake(AngularVelocity velocity) {
        io.intake(velocity);
    }
}
