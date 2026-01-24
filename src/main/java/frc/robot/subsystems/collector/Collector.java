// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Collector extends SubsystemBase {

  private CollectorModule[] collectors;
  private int downCollector;

  public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO) {
    collectors[0] = new CollectorModule(leftIO);
    collectors[1] = new CollectorModule(rightIO);

    downCollector = -1; // 0 == left collector mod, 1 == right collector mod
  }

  public String getDownCollector() {

    switch (downCollector) {
      case 0:
        return "LEFT";

      case 1:
        return "RIGHT";

      default:
        return null;
    }
  }

  public void setDownCollector(String downed) {
    switch (downed) {
      case "LEFT":
        downCollector = 0;
        break;

      case "RIGHT":
        downCollector = 1;
        break;

      default:
        break;
    }
  }

  public CollectorModule getLeftCollector() {
    return collectors[0];
  }

  public CollectorModule getRightCollector() {
    return collectors[1];
  }
}
