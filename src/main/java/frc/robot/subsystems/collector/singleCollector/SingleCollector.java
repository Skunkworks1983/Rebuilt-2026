// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.singleCollector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * This is the collector Subsystem
 *
 * <ul>
 *   <li>It takes in two collector module IO's to assign to two collector modules
 * </ul>
 */
public class SingleCollector extends SubsystemBase {

  @AutoLogOutput(key = "collectorSingle/mechanism")
  private LoggedMechanism2d loggedMechanism2d = new LoggedMechanism2d(0, 0);

  LoggedMechanismRoot2d root =
      loggedMechanism2d.getRoot(
          "collector",
          CollectorConstants.collectorLeftOriginTranslation.getX(),
          CollectorConstants.collectorLeftOriginTranslation.getY());
  LoggedMechanismLigament2d collectorLigament;

  private CollectorModuleSingle singleCollector;

  /**
   * Collector subsystem.
   *
   * <p><hl>
   * <li>Takes in a collector module IO </hl>
   *
   *     <p><img src="https://media.giphy.com/media/jVFgWDtkY5v2M/giphy.gif" alt="Our Collector">
   *
   * @param collector
   */
  public SingleCollector(CollectorModuleSingle collector) {
    this.singleCollector = collector;
  }

  @Override
  public void periodic() {
    this.singleCollector.periodic();
  }

  /**
   * @return - The Collector
   */
  public CollectorModuleSingle getSingleCollector() {
    return singleCollector;
  }
}
