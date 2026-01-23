// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

/** Add your docs here. */
public class Collector {
  private CollectorModule[] collectors;

  public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO) {
    collectors[0] = new CollectorModule(leftIO);
    collectors[2] = new CollectorModule(rightIO);
  }
}
