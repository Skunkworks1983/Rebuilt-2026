// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IndexerIOSim implements IndexerIO {
  DCMotorSim indexerMotorSim;
  Voltage voltage;

  public IndexerIOSim() {
    LinearSystem linearSystem = LinearSystemId.createDCMotorSystem(12.45, 0.07);
    double[] stdevs = {0.001, 0.001};
    indexerMotorSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX44(1), stdevs);
  }

  @Override
  public void updateInputs(IndexerIOInputs input) {
    indexerMotorSim.update(0.020);

    input.connected = true;
    input.position = indexerMotorSim.getAngularPosition();
    input.angularVelocity = indexerMotorSim.getAngularVelocity();
    input.appliedVolts = Units.Volts.of(indexerMotorSim.getInputVoltage());
    input.currentAmps = Units.Amps.of(indexerMotorSim.getCurrentDrawAmps());
  }

  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerMotorSim.setAngularVelocity(velocity.in(RadiansPerSecond));
  }
}
