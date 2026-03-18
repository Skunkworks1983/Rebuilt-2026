// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector.singleCollector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * This CollectorIO pretains to a talonFX collector, it has the functions of raising lowering, and
 * of course collecting.
 *
 * @author Liam B
 */
public class CollectorModuleSingleTalonIO implements CollectorModuleSingleIO {
  // Motors and Hardware objects
  protected final TalonFX turnMotor;
  protected final TalonFX intakeMotor;
  protected final DigitalInput upperLimitSwitch;
  protected final DigitalInput lowerLimitSwitch;

  // Constants
  public final CollectorModuleConstants constants;

  // Helper variables
  private boolean setUpRunning;
  private boolean setDownRunning;
  private AngularVelocity lastVelocity;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Double> turnError;
  private final StatusSignal<Double> turnSetPoint;

  // Inputs from intake motor
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;
  private final LoggedTunableNumber collectorIntakeVelocity;

  // Connection debouncers
  private final Debouncer intakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Instanciates, configures, and sets up the IO inputs for motors
   *
   * @param constants left <h7> OR </h7> right constants
   */
  public CollectorModuleSingleTalonIO(CollectorModuleConstants constants) {
    // Constants
    this.constants = constants;

    // isRunning Variables
    setUpRunning = false;
    setDownRunning = false;

    // Instantiates all motors and hardware objects
    turnMotor = new TalonFX(constants.turnMotorID, constants.canbus);
    intakeMotor = new TalonFX(constants.intakeMotorID, constants.canbus);
    upperLimitSwitch = new DigitalInput(constants.UpperLimitChannel);
    lowerLimitSwitch = new DigitalInput(constants.LowerLimitChannel);
    turnMotor.setPosition(Degrees.of(0));

    // Config for turnMotor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = constants.turnSupplyCurrent;
    turnConfig.Feedback.SensorToMechanismRatio = constants.turnMotorGearRatio;
    turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Pid Configs for Turn Motor
    var turnSlot0Configs = turnConfig.Slot0;

    if (RobotBase.isReal()) {
      turnSlot0Configs.kP = constants.turnkP;
      turnSlot0Configs.kD = constants.turnkD;
      turnSlot0Configs.kI = constants.turnkI;
    } else if (RobotBase.isSimulation()) {
      turnSlot0Configs.kP = constants.SimTurnkP;
      turnSlot0Configs.kD = constants.SimTurnkD;
      turnSlot0Configs.kI = constants.SimTurnkI;
    } else {
      turnSlot0Configs.kP = constants.turnkP;
      turnSlot0Configs.kD = constants.turnkD;
      turnSlot0Configs.kI = constants.turnkI;
    }

    var turnSlot1Configs = turnConfig.Slot1;

    turnSlot1Configs.kP = constants.turnkPSlot1;
    turnSlot1Configs.kD = constants.turnkDSlot1;
    turnSlot1Configs.kI = constants.turnkISlot1;

    turnSlot0Configs.kS = constants.turnkS;
    turnSlot0Configs.kV = constants.turnkV;
    turnSlot0Configs.kA = constants.turnkA;

    turnSlot1Configs.kS = constants.turnkSSlot1;
    turnSlot1Configs.kV = constants.turnkVSlot1;
    turnSlot1Configs.kA = constants.turnkASlot1;

    // Motion Magic Configs for Turn Motor
    var turnMotionMagicConfig = turnConfig.MotionMagic;
    turnMotionMagicConfig.MotionMagicCruiseVelocity =
        constants.motionMagicCruiseVelocity.in(RotationsPerSecond);
    turnMotionMagicConfig.MotionMagicAcceleration =
        constants.motionMagicAcceleration.in(RotationsPerSecondPerSecond);

    // Applying Turn Motor Configs
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

    // PID config for intakeMotor // TODO: make config for lower intake
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = constants.intakeSupplyCurrent;
    intakeConfig.MotorOutput.Inverted = constants.intakeInvertedValue;

    if (RobotBase.isReal()) {
      intakeConfig.Slot0.kP = constants.intakekP;
      intakeConfig.Slot0.kD = constants.intakekP;
      intakeConfig.Slot0.kI = constants.intakekI;
      intakeConfig.Slot0.kV = constants.intakekV;
    } else if (RobotBase.isSimulation()) {
      intakeConfig.Slot0.kP = constants.SimIntakekP;
      intakeConfig.Slot0.kD = constants.SimIntakekP;
      intakeConfig.Slot0.kI = constants.SimIntakekI;
    }

    collectorIntakeVelocity =
        new LoggedTunableNumber(
            "Collector/Single/IntakeUpper/Velocity",
            constants.CollectorIntakeSpeed.baseUnitMagnitude());

    // Applying Intake Configs
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    // Create turn status signals
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();
    turnError = turnMotor.getClosedLoopError();
    turnSetPoint = turnMotor.getClosedLoopReference();

    // Create upper intake status signals
    intakePosition = intakeMotor.getPosition();
    intakeVelocity = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrent = intakeMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVelocity,
        intakeAppliedVolts,
        intakeCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnError,
        turnSetPoint,
        turnPosition,
        intakePosition);
    ParentDevice.optimizeBusUtilizationForAll(turnMotor, intakeMotor);
  }

  @Override
  /**
   * Updates our inputs for all motors
   *
   * @param inputs
   */
  public void updateInputs(CollectorIOSingleInputs inputs) {
    // Refresh all signals
    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, turnSetPoint);
    var upperStatus =
        BaseStatusSignal.refreshAll(
            intakePosition, intakeVelocity, intakeAppliedVolts, intakeCurrent);

    inputs.goingDown = setDownRunning;

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocity = turnVelocity.getValue();
    inputs.turnAppliedVolts = turnAppliedVolts.getValue();
    inputs.turnCurrentAmps = turnCurrent.getValue();
    inputs.turnClosedLoopError = turnError.getValueAsDouble();
    inputs.turnSetPoint = turnSetPoint.getValueAsDouble();

    // Update intake inputs
    inputs.intakeConnected = intakeConnectedDebounce.calculate(upperStatus.isOK());
    inputs.intakePositionRad = intakePosition.getValue();
    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValue();
    inputs.intakeCurrentAmps = intakeCurrent.getValue();

    // Update sensor inputs
    inputs.upperTriggered = !upperLimitSwitch.get();
    inputs.lowerTriggered = !lowerLimitSwitch.get();

    // up and down
    inputs.isUp =
        !upperLimitSwitch.get()
            || turnMotor
                .getPosition()
                .getValue()
                .lte(constants.CollectorUpPos.plus(constants.CollectorPosTolerance));
    inputs.isDown =
        !lowerLimitSwitch.get()
            || turnMotor
                .getPosition()
                .getValue()
                .gte(
                    Radians.of(constants.CollectorDownPos.get())
                        .minus(constants.CollectorPosTolerance));

    Logger.recordOutput(
        "Collector/Single/Collector Up Position",
        turnMotor
            .getPosition()
            .getValue()
            .lte(constants.CollectorUpPos.plus(constants.CollectorPosTolerance)));
  }

  /** Gets The Collector Constants */
  public CollectorModuleConstants getConstants() {
    return constants;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    if (!velocity.equals(lastVelocity)) {
      intakeMotor.setControl(
          new VelocityTorqueCurrentFOC(RotationsPerSecond.of(collectorIntakeVelocity.get())));
      lastVelocity = velocity;
    }
  }

  /** sets Just the encoder pos */
  public void setCollectorEncoderPos(Angle angle) {
    turnMotor.setPosition(angle);
  }

  /**
   * Sets the collectors position to the angle given
   *
   * @param angle -WPILib Angle
   */
  public void setCollectorPos(Angle angle, int slot) {
    turnMotor.setControl(new MotionMagicVoltage(angle).withSlot(slot));
  }

  /**
   * Sets the dutyCycle of our pivot motor collector motors
   *
   * @param dutyCycle -double between -1.0 and 1.0
   */
  public void setCollectorPivotDutyCycle(double dutyCycle) {
    turnMotor.setControl(new DutyCycleOut(dutyCycle));
  }

  /**
   * Sets the duty cycle of the intake motors
   *
   * @param dutyCycle double between -1.0 and 1.0
   */
  public void setCollectorIntakeDutyCycle(double dutyCycle) {
    intakeMotor.set(dutyCycle);
    lastVelocity = RotationsPerSecond.of(dutyCycle);
  }

  // Unknown if we need these for now

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   *
   * @return whether or not collector is down
   */
  public boolean isCollectorDown() {
    return !lowerLimitSwitch.get()
        || turnMotor
            .getPosition()
            .getValue()
            .gte(
                Radians.of(constants.CollectorDownPos.get())
                    .minus(constants.CollectorPosTolerance));
  }

  /**
   * Checks to see if the collector is up
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   *
   * @return whether or not collector is up
   */
  public boolean isCollectorUp() {
    return !upperLimitSwitch.get()
        || turnMotor
            .getPosition()
            .getValue()
            .lte(constants.CollectorUpPos.plus(constants.CollectorPosTolerance));
  }

  /** Sets Collector to its up position */
  public void goUp() {
    if (!setUpRunning) {
      setCollectorPos(constants.CollectorUpPos, 0);
      Logger.recordOutput(
          "Collector/" + constants.turnMotorID + "/setpoint", constants.CollectorUpPos);
      setUpRunning = true;
      setDownRunning = false;
    }
  }

  /** Sets Collector to its up position */
  public void goDown() {
    if (!setDownRunning) {
      setCollectorPos(Radians.of(constants.CollectorDownPos.get()), 0);
      Logger.recordOutput(
          "Collector/" + constants.turnMotorID + "/setpoint", constants.CollectorDownPos);

      setDownRunning = true;
      setUpRunning = false;
    }
  }

  @Override
  public void setPivotGains(Slot0Configs slotConfigs) {
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(slotConfigs, 0.25));
  }

  @Override
  public void setIntakeGains(Slot0Configs slotConfigs) {
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(slotConfigs, 0.25));
  }

  @Override
  public void setMotionConstraints(MotionMagicConfigs motionMagicConfigs) {
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(motionMagicConfigs, 0.25));
  }
}
