// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants.Side;

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

  // Constants
  private final CollectorModuleConstants constants;

  // Helper variables
  private boolean isDown;
  private Side side;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Double> turnError;

  // Inputs from lower intake motor
  private final StatusSignal<Angle> lowerPosition;
  private final StatusSignal<AngularVelocity> lowerVelocity;
  private final StatusSignal<Voltage> lowerAppliedVolts;
  private final StatusSignal<Current> lowerCurrent;

  // Inputs from upper intake motor
  private final StatusSignal<Angle> upperPosition;
  private final StatusSignal<AngularVelocity> upperVelocity;
  private final StatusSignal<Voltage> upperAppliedVolts;
  private final StatusSignal<Current> upperCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Instanciates, configures, and sets up the IO inputs for motors */
  public CollectorModuleTalonIO(CollectorModuleConstants constants) {
    // Constants
    this.constants = constants;

    // Helper variables
    isDown = false;

    // Collector side
    side = constants.collectorSide;

    // Instantiates all motors and hardware objects
    turnMotor = new TalonFX(constants.turnMotorID, TunerConstants.getTunerConstants().getCANBus());
    intakeMotorLower =
        new TalonFX(constants.intakeMotorLowerID, TunerConstants.getTunerConstants().getCANBus());
    intakeMotorUpper =
        new TalonFX(constants.intakeMotorUpperID, TunerConstants.getTunerConstants().getCANBus());

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

    // Inverts left motor
    if (side.equals(Side.LEFT)) {
      turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

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

    // Create turn status signals
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();
    turnError = turnMotor.getClosedLoopError();

    // Create drive status signals
    lowerPosition = intakeMotorLower.getPosition();
    lowerVelocity = intakeMotorLower.getVelocity();
    lowerAppliedVolts = intakeMotorLower.getMotorVoltage();
    lowerCurrent = intakeMotorLower.getStatorCurrent();

    // Create drive status signals
    upperPosition = intakeMotorUpper.getPosition();
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
        turnCurrent,
        turnError);
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
    inputs.turnVelocity = turnVelocity.getValue();
    inputs.turnAppliedVolts = turnAppliedVolts.getValue();
    inputs.turnCurrentAmps = turnCurrent.getValue();
    inputs.turnClosedLoopError = turnError.getValueAsDouble();

    // Update lower intake inputs
    inputs.lowerConnected = driveConnectedDebounce.calculate(lowerStatus.isOK());
    inputs.lowerPositionRad = lowerPosition.getValue();
    inputs.lowerVelocity = lowerVelocity.getValue();
    inputs.lowerAppliedVolts = lowerAppliedVolts.getValue();
    inputs.lowerCurrentAmps = lowerCurrent.getValue();

    // Update upper intake inputs
    inputs.upperConnected = driveConnectedDebounce.calculate(upperStatus.isOK());
    inputs.upperPositionRad = upperPosition.getValue();
    inputs.upperVelocity = upperVelocity.getValue();
    inputs.upperAppliedVolts = upperAppliedVolts.getValue();
    inputs.upperCurrentAmps = upperCurrent.getValue();
  }

  /** Gets Constants */
  public CollectorModuleConstants getConstants() {
    return constants;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    intakeMotorLower.setControl(new VelocityVoltage(velocity));
    intakeMotorUpper.setControl(new VelocityVoltage(velocity));
  }

  /**
   * Sets the collectors position
   *
   * @param angle -WPILib Angle
   */
  public void setCollectorPos(Angle angle) {
    turnMotor.setControl(new MotionMagicVoltage(angle).withSlot(0));
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorDown() {
    return turnMotor.getPosition().getValue().gte(constants.CollectorDownPos.minus(Degrees.of(10)));
  }

  /**
   * Checks to see if the collector is up
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorUp() {
    return turnMotor.getPosition().getValue().lte(constants.CollectorUpPos.plus(Degrees.of(10)));
  }

  // Gets side i.e: left
  public Side getSide() {
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
