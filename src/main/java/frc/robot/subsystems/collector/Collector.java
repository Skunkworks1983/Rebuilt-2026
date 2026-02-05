// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the collector Subsystem
 *
 * <ul>
 *   <li>It takes in two collector module IO's to assign to two collector modules
 * </ul>
 */
public class Collector extends SubsystemBase {

  private CollectorModule[] collectors = new CollectorModule[2];

  public static enum State {
    MANUAL,
    AUTOMATIC,
    ALWAYSUP;
  }

  State state = State.MANUAL; // TODO: switch back to always up

  /**
   * Collector subsystem.
   *
   * <ul>
   *   <li>Takes in two collector module IOs and assigns them to the collector modules.
   * </ul>
   *
   * <p><img src="https://media.giphy.com/media/jVFgWDtkY5v2M/giphy.gif" alt="Our Collector">
   *
   * @param leftIO left collector
   * @param rightIO right collector
   */
  public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO) {
    collectors[0] = new CollectorModule(leftIO);
    collectors[1] = new CollectorModule(rightIO);
  }

  /**
   * @return -The current down collector
   */
  public String getDownCollector() {
    if (getLeftCollector().isCollectorDown() && getRightCollector().isCollectorDown()) {
      return "BOTH";
    } else if (getLeftCollector().isCollectorDown()) {
      return "LEFT";
    } else if (getRightCollector().isCollectorDown()) {
      return "RIGHT";
    } else {
      return "NEITHER";
    }
  }

  /**
   * @return -The current up collector
   */
  public String getUpCollector() {
    if (!getLeftCollector().isCollectorDown() && !getLeftCollector().isCollectorDown()) {
      return "BOTH";
    } else if (!getLeftCollector().isCollectorDown()) {
      return "LEFT";
    } else if (!getRightCollector().isCollectorDown()) {
      return "RIGHT";
    } else {
      return "NEITHER";
    }
  }

  /**
   * @return -Left collector Module
   */
  public CollectorModule getLeftCollector() {
    return collectors[0];
  }

  /**
   * @return -Right collector module
   */
  public CollectorModule getRightCollector() {
    return collectors[1];
  }

  // Changes State
  public void changeState(State desiredState) {
    state = desiredState;
  }

  // Gets State
  public State getState() {
    return state;
  }
}
