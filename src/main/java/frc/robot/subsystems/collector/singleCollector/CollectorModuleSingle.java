// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.singleCollector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants.Side;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * This is the collector module subsystem
 *
 * <ul>
 *   <li>Uses IO's to switch between left and right<\li>
 *       <ul>
 *
 * @author Liam B
 */
public class CollectorModuleSingle {
  private final CollectorModuleSingleIO collectorIO;
  private final CollectorIOSingleInputsAutoLogged inputs = new CollectorIOSingleInputsAutoLogged();

  private final LoggedTunableNumber collectorPivotKP;
  private final LoggedTunableNumber collectorPivotKD;
  private final LoggedTunableNumber collectorPivotKS;
  private final LoggedTunableNumber collectorPivotKV;
  private final LoggedTunableNumber collectorPivotKA;
  private final LoggedTunableNumber collectorPivotCruiseVelocity;
  private final LoggedTunableNumber collectorPivotMaxAcceleration;

  private final LoggedTunableNumber collectorIntakeKV;
  private final LoggedTunableNumber collectorIntakeKP;
  private final LoggedTunableNumber collectorIntakeKS;
  private final LoggedTunableNumber collectorIntakeKA;
  private final LoggedTunableNumber collectorIntakeVelocity;

  /**
   * This is the collector module subsystem
   *
   * <ul>
   *   <li>Uses IO's to switch between left and right
   * </ul>
   *
   * @param collectorIO
   */
  public CollectorModuleSingle(CollectorModuleSingleIO collectorIO) {
    this.collectorIO = collectorIO;
    String modulePrefix = "Collector/" + collectorIO.getSide() + "/";
    collectorPivotKP =
        new LoggedTunableNumber(modulePrefix + "Pivot/kP", collectorIO.getConstants().turnkP);
    collectorPivotKD =
        new LoggedTunableNumber(modulePrefix + "Pivot/kD", collectorIO.getConstants().turnkD);
    collectorPivotKS =
        new LoggedTunableNumber(modulePrefix + "Pivot/kS", collectorIO.getConstants().turnkS);
    collectorPivotKV =
        new LoggedTunableNumber(modulePrefix + "Pivot/kV", collectorIO.getConstants().turnkV);
    collectorPivotKA =
        new LoggedTunableNumber(modulePrefix + "Pivot/kA", collectorIO.getConstants().turnkA);

    collectorPivotCruiseVelocity =
        new LoggedTunableNumber(
            modulePrefix + "Pivot/CruiseVelocity [Rots]",
            collectorIO.getConstants().motionMagicCruiseVelocity.in(RotationsPerSecond));
    collectorPivotMaxAcceleration =
        new LoggedTunableNumber(
            modulePrefix + "Pivot/MaxAcceleration [Rots^2]",
            collectorIO.getConstants().motionMagicAcceleration.in(RotationsPerSecondPerSecond));

    collectorIntakeKP =
        new LoggedTunableNumber(modulePrefix + "Intake/kP", collectorIO.getConstants().intakekP);

    collectorIntakeKV =
        new LoggedTunableNumber(modulePrefix + "Intake/kV", collectorIO.getConstants().intakekV);
    collectorIntakeKA =
        new LoggedTunableNumber(modulePrefix + "Intake/kA", collectorIO.getConstants().intakekA);
    collectorIntakeKS =
        new LoggedTunableNumber(modulePrefix + "Intake/kS", collectorIO.getConstants().intakekS);
    collectorIntakeVelocity =
        new LoggedTunableNumber(
            modulePrefix + "Intake/Velocity",
            collectorIO.getConstants().CollectorIntakeSpeed.baseUnitMagnitude());
  }

  public void periodic() {
    collectorIO.updateInputs(inputs);
    Logger.processInputs("Collector/Module/ " + collectorIO.getSide(), inputs);

    if (Constants.tuningMode) {
      int hash = hashCode();
      if (collectorPivotKP.hasChanged(hash)
          || collectorPivotKD.hasChanged(hash)
          || collectorPivotKS.hasChanged(hash)
          || collectorPivotKV.hasChanged(hash)
          || collectorPivotKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = collectorPivotKP.get();
        gains.kD = collectorPivotKD.get();
        gains.kS = collectorPivotKS.get();
        gains.kV = collectorPivotKV.get();
        gains.kA = collectorPivotKA.get();

        collectorIO.setPivotGains(gains);
      }

      if (collectorIntakeKP.hasChanged(hash)
          || collectorIntakeKV.hasChanged(hash)
          || collectorIntakeVelocity.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = collectorIntakeKP.get();
        gains.kV = collectorIntakeKV.get();

        collectorIO.setIntakeGains(gains);
      }

      if (collectorPivotCruiseVelocity.hasChanged(hash)
          || collectorPivotMaxAcceleration.hasChanged(hash)) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = collectorPivotCruiseVelocity.get();
        motionMagicConfigs.MotionMagicAcceleration = collectorPivotMaxAcceleration.get();
        collectorIO.setMotionConstraints(motionMagicConfigs);
      }
    }
  }

  /** Gets Constants */
  public CollectorModuleConstants getConstants() {
    return collectorIO.getConstants();
  }

  /**
   * Gets the position of the collector
   *
   * @return -WPILib Angle
   */
  public Angle getPosition() {
    return inputs.turnPosition.getMeasure();
  }

  /**
   * Gets the velocity of the collectors intake motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfIntake() {
    return inputs.intakeUpperVelocity;
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfTurn() {
    return inputs.turnVelocity;
  }

  /**
   * Sets the velocity of our collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    collectorIO.setCollectorVelocity(velocity);
  }

  /** sets Just the encoder pos */
  public void setCollectorEncoderPos(Angle angle) {
    collectorIO.setCollectorEncoderPos(angle);
  }

  /**
   * Sets the collectors position
   *
   * @param angle WPILib angle
   */
  public void setCollectorPos(Angle angle, int slot) {
    collectorIO.setCollectorPos(angle, slot);
  }

  /**
   * Sets the velocity of our pivot motor collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorPivotDutyCycle(double dutyCycle) {
    collectorIO.setCollectorPivotDutyCycle(dutyCycle);
  }

  /**
   * Sets the duty cycle of the intake motors
   *
   * @param dutyCycle double between -1.0 and 1.0
   */
  public void setCollectorDutyCycle(double dutyCycle) {
    collectorIO.setCollectorIntakeDutyCycle(dutyCycle);
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorDown() {
    return collectorIO.isCollectorDown();
  }

  /**
   * Checks to see if the collector is up
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorUp() {
    return collectorIO.isCollectorUp();
  }

  // Sets collector up
  public void goUp() {
    collectorIO.goUp();
  }

  // Sets collector down
  public void goDown() {
    collectorIO.goDown();
  }

  // Gets collector side
  public Side getSide() {
    return collectorIO.getSide();
  }
}
