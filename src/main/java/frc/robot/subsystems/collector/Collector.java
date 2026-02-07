// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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

  private CollectorModule[] collectors = new CollectorModule[2];

  public static enum State {
    MANUAL,
    AUTOMATIC,
    ALWAYSUP;
  }

  State state = State.MANUAL;

  /**
   * Collector subsystem.
   *
   * <ul>
   *   <li>Takes in two collector module IOs and assigns them to the collector modules.
   * </ul>
   *
   * <p><img src="https://media.giphy.com/media/jVFgWDtkY5v2M/giphy.gif" alt="Our Collector 💕">
   *
   * @param leftIO left collector
   * @param rightIO right collector
   */
  public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO) {
    collectors[0] = new CollectorModule(leftIO);
    collectors[1] = new CollectorModule(rightIO);

    collectorLigamentLeft =
        rootLeft.append(
            new LoggedMechanismLigament2d("collector body left", Meters.of(0.25), Degrees.of(90)));
    collectorLigamentRight =
        rootRight.append(
            new LoggedMechanismLigament2d("collector body right", Meters.of(0.25), Degrees.of(90)));
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

  // Sound feedback 😈
  public void playSound(String file) {
    collectors[0].playSound(file);
    collectors[1].playSound(file);
  }

  // Stop sound feedback
  public void stopSound() {
    collectors[0].stopSound();
    collectors[1].stopSound();
  }

  // is sound playing?
  public boolean isSoundPlaying() {
    if (collectors[0].isSoundPlaying() || collectors[0].isSoundPlaying()) {
      return true;
    } else {
      return false;
    }
  }

  // Changes State
  public void changeState(State desiredState) {
    state = desiredState;
  }

  // Gets State
  public State getState() {
    return state;
  }

  @Override
  public void periodic() {
    collectorLigamentLeft.setAngle(getLeftCollector().getPosition());
    collectorLigamentRight.setAngle(getRightCollector().getPosition());
  }
}
