// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  private final CollectorIO bottomCollectorMotor;
  private final CollectorIO topCollectorMotor;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  public Collector(CollectorIO bottomCollectorMotor, CollectorIO topCollectoroMotor) {
    this.bottomCollectorMotor = bottomCollectorMotor;
    this.topCollectorMotor = topCollectoroMotor;
  }

  // motor ID 11

  @Override
  public void periodic() {
    bottomCollectorMotor.updateInputs(inputs);
    topCollectorMotor.updateInputs(inputs);
  }

  public void setCollectorVelocity(double bottomMotorSpeed, double topMotorSpeed) {
    bottomCollectorMotor.setCollectorVelocity(bottomMotorSpeed);
    topCollectorMotor.setCollectorVelocity(topMotorSpeed);
  }
}
