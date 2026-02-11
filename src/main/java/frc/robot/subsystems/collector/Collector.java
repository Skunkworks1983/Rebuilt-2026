// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.collectorCommands.AutomaticStateChange;
import frc.robot.commands.collectorCommands.BothCollectorsUpState;
import java.util.function.DoubleSupplier;

/**
 * This is the collector Subsystem
 *
 * <ul>
 *   <li>It takes in two collector module IO's to assign to two collector modules
 * </ul>
 */
public class Collector extends SubsystemBase {

  private CollectorModule leftCollector;
  private CollectorModule rightCollector;

  public static enum Mode {
    MANUAL,
    AUTOMATIC,
    ALWAYSUP;
  }

  Mode mode = Mode.ALWAYSUP;

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
    // Collector Modules
    leftCollector = new CollectorModule(leftIO);
    rightCollector = new CollectorModule(rightIO);
  }

  @Override
  public void periodic() {
    leftCollector.periodic();
    rightCollector.periodic();
  }

  /**
   * @return -Left collector Module
   */
  public CollectorModule getLeftCollector() {
    return leftCollector;
  }

  /**
   * @return -Right collector module
   */
  public CollectorModule getRightCollector() {
    return rightCollector;
  }
}
