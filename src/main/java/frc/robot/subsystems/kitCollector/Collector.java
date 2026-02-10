// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Collector subsystem — brings in the collector I/O and logs inputs
 * Use this to run the intake motor and monitor its sensors.
 */
public class Collector extends SubsystemBase {

  private final CollectorIO io;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  public Collector(CollectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // refresh inputs from hardware and feed to logger 
    io.updateInputs(inputs); 
    Logger.processInputs(getName(), inputs);
  }

  public void intake(AngularVelocity velocity) {
    // command the collector to intake at the requested velocity 
    io.intake(velocity); // go collect!
  }
}
