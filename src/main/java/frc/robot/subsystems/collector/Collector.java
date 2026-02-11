// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.collectorCommands.AutomaticStateChange;
import frc.robot.commands.collectorCommands.BothCollectorsUpState;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import java.util.function.DoubleSupplier;
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
public class Collector extends SubsystemBase {

  @AutoLogOutput(key = "collectorRight/mechanism")
  private LoggedMechanism2d loggedMechanism2dLeft = new LoggedMechanism2d(0, 0);

  LoggedMechanismRoot2d rootLeft =
      loggedMechanism2dLeft.getRoot(
          "collector",
          CollectorConstants.collectorLeftOriginTranslation.getX(),
          CollectorConstants.collectorLeftOriginTranslation.getY());
  LoggedMechanismLigament2d collectorLigamentLeft;

  @AutoLogOutput(key = "collectorLeft/mechanism")
  private LoggedMechanism2d loggedMechanism2dRight = new LoggedMechanism2d(0, 0);

  LoggedMechanismRoot2d rootRight =
      loggedMechanism2dRight.getRoot(
          "collector",
          CollectorConstants.collectorRightOriginTranslation.getX(),
          CollectorConstants.collectorRightOriginTranslation.getY());
  LoggedMechanismLigament2d collectorLigamentRight;

  private CollectorModule leftCollector;
  private CollectorModule rightCollector;

  public static enum Mode {
    MANUAL,
    AUTOMATIC,
    ALWAYSUP;
  }

  Mode mode = Mode.MANUAL;

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
    leftCollector = new CollectorModule(leftIO); // lllllllllllll
    rightCollector = new CollectorModule(rightIO);

    collectorLigamentLeft =
        rootLeft.append(
            new LoggedMechanismLigament2d("collector body left", Meters.of(0.25), Degrees.of(90)));
    collectorLigamentRight =
        rootRight.append(
            new LoggedMechanismLigament2d("collector body right", Meters.of(0.25), Degrees.of(90)));
  }

  @Override
  public void periodic() {
    leftCollector.periodic();
    rightCollector.periodic();
    collectorLigamentLeft.setAngle(getLeftCollector().getPosition());
    collectorLigamentRight.setAngle(getRightCollector().getPosition());
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

  // Changes State
  public void changeMode(Mode desiredMode) {
    mode = desiredMode;
    // Collector State Control
    switch (desiredMode) {
      case ALWAYSUP:
        new BothCollectorsUpState(this);
        break;

      case MANUAL:
        break;

      case AUTOMATIC:
        new AutomaticStateChange(this, direction);
        break;

      default:
        break;
    }
  }

  // Gets State
  public Mode getMode() {
    return mode;
  }
}
