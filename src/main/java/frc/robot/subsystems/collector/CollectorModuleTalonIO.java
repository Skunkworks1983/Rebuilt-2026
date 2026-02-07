// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.side;
import frc.robot.subsystems.collector.collectorConstants.CollectorPositionConstants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import java.util.Queue;

/**
 * This CollectorIO pretains to a talonFX collector, it has the functions of raising lowering, and
 * of course collecting.
 *
 * @author Liam B
 */
public class CollectorModuleTalonIO implements CollectorModuleIO {
  // Motors and Hardware objects
  protected final TalonFX turnMotor;
  protected final TalonFX intakeMotorLower;
  protected final TalonFX intakeMotorUpper;

  // 🤫
  private Orchestra sound;

  // Constants
  private final CollectorConstants constants;

  // Helper variables
  private boolean isDown;
  private side side;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Inputs from lower intake motor
  private final StatusSignal<Angle> lowerPosition;
  private final Queue<Double> lowerPositionQueue;
  private final StatusSignal<AngularVelocity> lowerVelocity;
  private final StatusSignal<Voltage> lowerAppliedVolts;
  private final StatusSignal<Current> lowerCurrent;

  // Inputs from upper intake motor
  private final StatusSignal<Angle> upperPosition;
  private final Queue<Double> upperPositionQueue;
  private final StatusSignal<AngularVelocity> upperVelocity;
  private final StatusSignal<Voltage> upperAppliedVolts;
  private final StatusSignal<Current> upperCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Instanciates, configures, and sets up the IO inputs for motors */
  public CollectorModuleTalonIO(CollectorConstants constants) {
    // Constants
    this.constants = constants;

    // Helper variables
    isDown = false;

    // Collector side
    side = constants.collectorSide;

    // Instantiates all motors and hardware objects
    turnMotor = new TalonFX(constants.turnMotorID);
    intakeMotorLower = new TalonFX(constants.intakeMotorLowerID);
    intakeMotorUpper = new TalonFX(constants.intakeMotorUpperID);

    // 👀👀
    sound = new Orchestra();
    sound.addInstrument(turnMotor);
    sound.addInstrument(intakeMotorLower);
    sound.addInstrument(intakeMotorUpper);

    // Config for turnMotor
    var turnConfig = new TalonFXConfiguration();

    // Pid Configs for Turn Motor
    var turnSlot0Configs = turnConfig.Slot0;
    turnSlot0Configs.kP = constants.turnkP;
    turnSlot0Configs.kD = constants.turnkD;
    turnSlot0Configs.kI = constants.turnkI;
    turnSlot0Configs.kS = constants.turnkS;
    turnSlot0Configs.kV = constants.turnkV;
    turnSlot0Configs.kA = constants.turnkA;

    // Motion Magic Configs for Turn Motor
    var turnMotionMagicConfig = turnConfig.MotionMagic;
    turnMotionMagicConfig.MotionMagicCruiseVelocity = 80;
    turnMotionMagicConfig.MotionMagicAcceleration = 160;

    // Applying Turn Motor Configs
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

    // PID config for intakeMotorLower
    var lowerConfig = new TalonFXConfiguration();
    lowerConfig.Slot0.kP = constants.lowerkP;
    lowerConfig.Slot0.kD = constants.lowerkD;
    lowerConfig.Slot0.kI = constants.lowerkI;

    // Applying Lower Intake Configs
    tryUntilOk(5, () -> intakeMotorLower.getConfigurator().apply(lowerConfig, 0.25));

    // PID config for intakeMotorUpper
    var upperConfig = new TalonFXConfiguration();
    upperConfig.Slot0.kP = constants.upperkP;
    upperConfig.Slot0.kD = constants.upperkP;
    upperConfig.Slot0.kI = constants.upperkI;

    // Applying Upper Intake Configs
    tryUntilOk(5, () -> intakeMotorUpper.getConfigurator().apply(upperConfig, 0.25));

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create turn status signals
    turnPosition = turnMotor.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();

    // Create drive status signals
    lowerPosition = intakeMotorLower.getPosition();
    lowerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(lowerPosition.clone());
    lowerVelocity = intakeMotorLower.getVelocity();
    lowerAppliedVolts = intakeMotorLower.getMotorVoltage();
    lowerCurrent = intakeMotorLower.getStatorCurrent();

    // Create drive status signals
    upperPosition = intakeMotorUpper.getPosition();
    upperPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(upperPosition.clone());
    upperVelocity = intakeMotorUpper.getVelocity();
    upperAppliedVolts = intakeMotorUpper.getMotorVoltage();
    upperCurrent = intakeMotorUpper.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnPosition, lowerPosition, upperPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        upperVelocity,
        upperAppliedVolts,
        upperCurrent,
        lowerVelocity,
        lowerAppliedVolts,
        lowerCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(turnMotor, intakeMotorLower, intakeMotorUpper);
  }

  @Override
  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  public void updateInputs(CollectorIOInputs inputs) {
    // Refresh all signals
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var lowerStatus =
        BaseStatusSignal.refreshAll(lowerPosition, lowerVelocity, lowerAppliedVolts, lowerCurrent);
    var upperStatus =
        BaseStatusSignal.refreshAll(upperPosition, upperVelocity, upperAppliedVolts, upperCurrent);

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update lower intake inputs
    inputs.lowerConnected = driveConnectedDebounce.calculate(lowerStatus.isOK());
    inputs.lowerPositionRad = Units.rotationsToRadians(lowerPosition.getValueAsDouble());
    inputs.lowerVelocityRadPerSec = Units.rotationsToRadians(lowerVelocity.getValueAsDouble());
    inputs.lowerAppliedVolts = lowerAppliedVolts.getValueAsDouble();
    inputs.lowerCurrentAmps = lowerCurrent.getValueAsDouble();

    // Update upper intake inputs
    inputs.upperConnected = driveConnectedDebounce.calculate(upperStatus.isOK());
    inputs.upperPositionRad = Units.rotationsToRadians(upperPosition.getValueAsDouble());
    inputs.upperVelocityRadPerSec = Units.rotationsToRadians(upperVelocity.getValueAsDouble());
    inputs.upperAppliedVolts = upperAppliedVolts.getValueAsDouble();
    inputs.upperCurrentAmps = upperCurrent.getValueAsDouble();
  }

  /**
   * Gets the position of the collector
   *
   * <ul>
   *   <img
   *   src="https://media1.giphy.com/media/v1.Y2lkPTc5MGI3NjExYzBwYmd1MHEwMWZhejhubWY2emZ6Nmd3ZTkxN294eXFtcDBvaTM5byZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/rUxSaLgjcQbLO/giphy.gif"
   *   alt="Intake Lower🤔">
   *
   * @return -WPILib Angle
   */
  public Angle getPosition() {
    return turnMotor.getPosition().getValue();
  }

  /**
   * Gets the velocity of the collectors intake upper motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfIntakeLower() {
    return intakeMotorLower.getVelocity().getValue();
  }

  /**
   * Gets the velocity of the collectors intake lower motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfIntakeUpper() {
    return intakeMotorUpper.getVelocity().getValue();
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return -Angular Velocity
   */
  public AngularVelocity getVelocityOfTurn() {
    return turnMotor.getVelocity().getValue();
  }

  /**
   * Gets the turn motors gear ratio
   *
   * @return -Turn motors gear ratio
   */
  public double getTurnGearRatio() {
    return constants.turnMotorGearRatio;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {}

  /**
   * Sets the collectors position
   *
   * @param angle -WPILib Angle
   */
  public void setCollectorPos(Angle angle) {
    turnMotor.setControl(new PositionVoltage(angle).withSlot(0));
  }

  /**
   * Sets the collectors turn motor velocity
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorTurnMotorVelocity(AngularVelocity velocity) {
    turnMotor.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorDown() {
    switch (side) {
      case LEFT:
        if (getPosition().in(Degrees)
            <= CollectorPositionConstants.leftCollectorDownPos.in(Degrees)) {
          return true;
        } else {
          return false;
        }

      case RIGHT:
        if (getPosition().in(Degrees)
            <= CollectorPositionConstants.rightCollectorDownPos.in(Degrees)) {
          return true;
        } else {
          return false;
        }
      default:
        return false;
    }
  }

  // Sound feedback 😈
  public void playSound(String file) {
    sound.loadMusic(file);
    sound.play();
  }

  // Stop sound feedback
  public void stopSound() {
    if (sound.isPlaying()) {
      sound.stop();
    }
  }

  // is sound playing?
  public boolean isSoundPlaying() {
    if (sound.isPlaying()) {
      return true;
    } else {
      return false;
    }
  }

  // Gets side i.e: left
  public side getSide() {
    return side;
  }

  // Sets collector up
  public void goUp() {
    setCollectorPos(constants.CollectorUpPos);
  }

  // Sets collector down
  public void goDown() {
    setCollectorPos(constants.CollectorDownPos);
  }
}
