// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** This is the Trigger IO sim */
public class TriggerIOSim extends TriggerIOTalonFXS {
  DCMotorSim triggerMotorSim;
  Voltage voltage;

  public TriggerIOSim(int motorID) {
    super(motorID);
    LinearSystem linearSystem = LinearSystemId.createDCMotorSystem(0.002, 0.02);

    double[] stdevs = {0.00, 0.00};

    triggerMotorSim = new DCMotorSim(linearSystem, DCMotor.getMinion(1), stdevs);
  }

  @Override
  public void updateInputs(TriggerIOInputs input) {
    var talonFXSim = super.triggerMotor.getSimState();

    talonFXSim.MotorOrientation = ChassisReference.Clockwise_Positive;
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    Voltage primaryAppliedVolts = talonFXSim.getMotorVoltageMeasure();
    triggerMotorSim.setInputVoltage(primaryAppliedVolts.magnitude());

    triggerMotorSim.update(0.020);

    talonFXSim.setRotorVelocity(triggerMotorSim.getAngularVelocity());

    talonFXSim.setRawRotorPosition(triggerMotorSim.getAngularPosition());

    triggerMotorSim.update(0.020);

    input.connected = true;
    input.position = triggerMotorSim.getAngularPosition();
    input.angularVelocity = triggerMotorSim.getAngularVelocity();
    input.appliedVolts = primaryAppliedVolts;
    input.currentAmps = Units.Amps.of(triggerMotorSim.getCurrentDrawAmps());
  }
}
