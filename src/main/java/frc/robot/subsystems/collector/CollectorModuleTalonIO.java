// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.Side;
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
  private final TalonFX turnMotor;
  private final TalonFX intakeMotorLower;
  private final TalonFX intakeMotorUpper;

  // Constants
  private final CollectorConstants constants;

  // Helper variables
  private boolean isDown;
  private Side side;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

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

    // Create turn status signals
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();

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

  /** Gets Constants */
  public CollectorConstants getConstants() {
    return constants;
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    intakeMotorLower.setControl(new VelocityVoltage(velocity).withSlot(0));
    intakeMotorUpper.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   */
  public boolean isCollectorDown() {
    return turnPosition.getValue().lte(constants.CollectorDownPos);
  }

  // Sets collector up
  public void goUp() {
    turnMotor.setControl(new PositionVoltage(constants.CollectorUpPos).withSlot(0));
  }

  // Sets collector down
  public void goDown() {
    turnMotor.setControl(new PositionVoltage(constants.CollectorDownPos).withSlot(0));
  }
}
