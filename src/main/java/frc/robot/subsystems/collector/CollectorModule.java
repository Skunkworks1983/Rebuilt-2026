// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
public class CollectorModule {
  private final CollectorModuleIO collectorIO;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  private final LoggedTunableNumber collectorPivotKP;
  private final LoggedTunableNumber collectorPivotKD;
  private final LoggedTunableNumber collectorPivotKS;
  private final LoggedTunableNumber collectorPivotKV;
  private final LoggedTunableNumber collectorPivotKA;
  private final LoggedTunableNumber collectorPivotCruiseVelocity;
  private final LoggedTunableNumber collectorPivotMaxAcceleration;

  /**
   * This is the collector module subsystem
   *
   * <ul>
   *   <li>Uses IO's to switch between left and right
   * </ul>
   *
   * @param collectorIO
   */
  public CollectorModule(CollectorModuleIO collectorIO) {
    this.collectorIO = collectorIO;
    collectorPivotKP =
        new LoggedTunableNumber("Collector/Pivot/kP", collectorIO.getConstants().turnkP);
    collectorPivotKD =
        new LoggedTunableNumber("Collector/Pivot/kD", collectorIO.getConstants().turnkD);
    collectorPivotKS =
        new LoggedTunableNumber("Collector/Pivot/kS", collectorIO.getConstants().turnkS);
    collectorPivotKV =
        new LoggedTunableNumber("Collector/Pivot/kV", collectorIO.getConstants().turnkV);
    collectorPivotKA =
        new LoggedTunableNumber("Collector/Pivot/kA", collectorIO.getConstants().turnkA);

    collectorPivotCruiseVelocity =
        new LoggedTunableNumber(
            "Collector/Pivot/CruiseVelocity [Rot/s]",
            collectorIO.getConstants().pivotMaxVelocity.in(RotationsPerSecond));
    collectorPivotMaxAcceleration =
        new LoggedTunableNumber(
            "Collector/Pivot/MaxAcceleration [Rot/s/s]",
            collectorIO.getConstants().pivotMaxAcceleration.in(RotationsPerSecondPerSecond));
  }

  public void periodic() {
    collectorIO.updateInputs(inputs);
    Logger.processInputs("Collector/Module/ " + collectorIO.getSide(), inputs);

    if (collectorPivotKP.hasChanged(hashCode())
        || collectorPivotKD.hasChanged(hashCode())
        || collectorPivotKS.hasChanged(hashCode())
        || collectorPivotKV.hasChanged(hashCode())
        || collectorPivotKA.hasChanged(hashCode())) {
      Slot0Configs gains = new Slot0Configs();
      gains.kP = collectorPivotKP.get();
      gains.kD = collectorPivotKD.get();
      gains.kS = collectorPivotKS.get();
      gains.kV = collectorPivotKV.get();
      gains.kA = collectorPivotKA.get();
      collectorIO.setPivotGains(gains);
    }

    if (collectorPivotCruiseVelocity.hasChanged(hashCode())
        || collectorPivotMaxAcceleration.hasChanged(hashCode())) {
      MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
      motionMagicConfigs.MotionMagicCruiseVelocity = collectorPivotCruiseVelocity.get();
      motionMagicConfigs.MotionMagicAcceleration = collectorPivotMaxAcceleration.get();
      collectorIO.setMotionConstraints(motionMagicConfigs);
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
    return inputs.intakeVelocity;
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

  /**
   * Sets the collectors position
   *
   * @param angle WPILib angle
   */
  public void setCollectorPos(Angle angle) {
    collectorIO.setCollectorPos(angle);
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
