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
import com.ctre.phoenix6.signals.ForwardLimitValue;
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
 * This CollectorIO pertains to a talonFX collector, it has the functions of raising lowering, and
 * of course collecting.
 *
 * @author Liam B
 */
public class CollectorModuleTalonIO implements CollectorModuleIO {
  // Motors and Hardware objects
  private final TalonFX turnMotor;
  private final TalonFX intakeMotorLower;
  private final TalonFX intakeMotorUpper;

  // Side
  private final Side side;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<ForwardLimitValue> turnUpperLimit;

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

  /** Instantiates, configures, and sets up the IO inputs for motors */
  public CollectorModuleTalonIO(Side side) {
    this.side = side;

    // Instantiate motors based on side
    if (side == Side.LEFT) {
      turnMotor = new TalonFX(CollectorConstants.LEFT_TURN_MOTOR_ID);
      intakeMotorLower = new TalonFX(CollectorConstants.LEFT_INTAKE_MOTOR_LOWER_ID);
      intakeMotorUpper = new TalonFX(CollectorConstants.LEFT_INTAKE_MOTOR_UPPER_ID);
    } else {
      turnMotor = new TalonFX(CollectorConstants.RIGHT_TURN_MOTOR_ID);
      intakeMotorLower = new TalonFX(CollectorConstants.RIGHT_INTAKE_MOTOR_LOWER_ID);
      intakeMotorUpper = new TalonFX(CollectorConstants.RIGHT_INTAKE_MOTOR_UPPER_ID);
    }

    // Config for turnMotor
    var turnConfig = new TalonFXConfiguration();

    // Pid Configs for Turn Motor
    var turnSlot0Configs = turnConfig.Slot0;
    turnSlot0Configs.kP = CollectorConstants.TURN_KP;
    turnSlot0Configs.kD = CollectorConstants.TURN_KD;
    turnSlot0Configs.kI = CollectorConstants.TURN_KI;
    turnSlot0Configs.kS = CollectorConstants.TURN_KS;
    turnSlot0Configs.kV = CollectorConstants.TURN_KV;
    turnSlot0Configs.kA = CollectorConstants.TURN_KA;

    // Motion Magic Configs for Turn Motor
    var turnMotionMagicConfig = turnConfig.MotionMagic;
    turnMotionMagicConfig.MotionMagicCruiseVelocity = 80;
    turnMotionMagicConfig.MotionMagicAcceleration = 160;

    // Applying Turn Motor Configs
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

    // PID config for intakeMotorLower
    var lowerConfig = new TalonFXConfiguration();
    lowerConfig.Slot0.kP = CollectorConstants.LOWER_KP;
    lowerConfig.Slot0.kD = CollectorConstants.LOWER_KD;
    lowerConfig.Slot0.kI = CollectorConstants.LOWER_KI;

    // Applying Lower Intake Configs
    tryUntilOk(5, () -> intakeMotorLower.getConfigurator().apply(lowerConfig, 0.25));

    // PID config for intakeMotorUpper
    var upperConfig = new TalonFXConfiguration();
    upperConfig.Slot0.kP = CollectorConstants.UPPER_KP;
    upperConfig.Slot0.kD = CollectorConstants.UPPER_KD;
    upperConfig.Slot0.kI = CollectorConstants.UPPER_KI;

    // Applying Upper Intake Configs
    tryUntilOk(5, () -> intakeMotorUpper.getConfigurator().apply(upperConfig, 0.25));

    // Create turn status signals
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();
    turnUpperLimit = turnMotor.getForwardLimit();

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
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnPosition, lowerPosition, upperPosition, turnUpperLimit);
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
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, turnUpperLimit);
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
    inputs.turnUpperLimitSwitch = turnUpperLimit.getValue() == ForwardLimitValue.ClosedToGround;

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
   * Sets the velocity of our intake collector motors
   *
   * @param velocity -Angular Velocity
   */
  public void setCollectorVelocity(AngularVelocity velocity) {
    intakeMotorLower.setControl(new VelocityVoltage(velocity).withSlot(0));
    intakeMotorUpper.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  /**
   * Checks to see if the collector is up using the upper limit switch
   */
  public boolean isCollectorUp() {
    return turnUpperLimit.getValue() == ForwardLimitValue.ClosedToGround;
  }

  /**
   * Zeros the collector position when limit switch is triggered.
   * This method should only be called when the limit switch is active.
   */
  public void zeroPosition() {
    // Only zero if limit switch is actually triggered for safety
    if (turnUpperLimit.getValue() == ForwardLimitValue.ClosedToGround) {
      turnMotor.setPosition(CollectorConstants.COLLECTOR_UP_POS);
    }
  }

  // Sets collector up
  public void goUp() {
    turnMotor.setControl(new PositionVoltage(CollectorConstants.COLLECTOR_UP_POS).withSlot(0));
  }

  // Sets collector down
  public void goDown() {
    turnMotor.setControl(new PositionVoltage(CollectorConstants.COLLECTOR_DOWN_POS).withSlot(0));
  }
}
