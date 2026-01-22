// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

/** This CollectorIO pretains to the back collector, it has the functions of raising
 *      lowering, and of course collecting.
 * 
 * @author Liam B
 */
public class CollectorBackIO implements CollectorIO {
    // Constants for collector
    public static class Constants {
        public static final int turnMotorID = 0;
        public static final int intakeMotorLowerID = 0;
        public static final int intakeMotorUpperID = 0;

        public static final double turnMotorGearRatio = 0.0;
    }

    // Motors and Hardware objects
    private final TalonFX turnMotor;
    private final TalonFX intakeMotorLower;
    private final TalonFX intakeMotorUpper;

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
    public CollectorBackIO() {
        // Instantiates all motors and hardware objects
        turnMotor = new TalonFX(Constants.turnMotorID);
        intakeMotorLower = new TalonFX(Constants.intakeMotorLowerID);
        intakeMotorUpper = new TalonFX(Constants.intakeMotorUpperID);

        // Config for turnMotor
        var turnConfig = new TalonFXConfiguration();

        // Pid Configs for Turn Motor
        var turnSlot0Configs = turnConfig.Slot0;
        turnSlot0Configs.kP = 0;
        turnSlot0Configs.kD = 0;
        turnSlot0Configs.kI = 0;
        turnSlot0Configs.kS = 0.25;
        turnSlot0Configs.kV = 0.12;
        turnSlot0Configs.kA = 0.01;

        // Motion Magic Configs for Turn Motor
        var turnMotionMagicConfig = turnConfig.MotionMagic;
        turnMotionMagicConfig.MotionMagicCruiseVelocity = 80;
        turnMotionMagicConfig.MotionMagicAcceleration = 160;

        // Applying Turn Motor Configs
        tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

        // PID config for intakeMotorLower
        var lowerConfig = new TalonFXConfiguration();
        lowerConfig.Slot0.kP = 0;
        lowerConfig.Slot0.kD = 0;
        lowerConfig.Slot0.kI = 0;

        // Applying Lower Intake Configs
        tryUntilOk(5, () -> intakeMotorLower.getConfigurator().apply(lowerConfig, 0.25));

        // PID config for intakeMotorUpper
        var upperConfig = new TalonFXConfiguration();
        upperConfig.Slot0.kP = 0;
        upperConfig.Slot0.kD = 0;
        upperConfig.Slot0.kI = 0;

        //Applying Upper Intake Configs
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
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, turnPosition, lowerPosition, upperPosition);
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
    /** Updates our inputs for all motors 
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

    /** Gets the position of the collector 
     * @return
    */
    public Angle getPosition() {
        return turnMotor.getPosition().getValue();
    }

    /** Gets the velocity of the collectors intake upper motor
     * @return
    */
    public AngularVelocity getVelocityOfIntakeLower() {
        return intakeMotorLower.getVelocity().getValue();
    }

    /** Gets the velocity of the collectors intake lower motor 
     * @return
    */
    public AngularVelocity getVelocityOfIntakeUpper() {
        return intakeMotorUpper.getVelocity().getValue();
    }

    /** Gets the velocity of the turn motor 
     * @return
    */
    public AngularVelocity getVelocityOfTurn() {
        return turnMotor.getVelocity().getValue();
    }

    /** Gets the turn motors gear ratio 
     * @return
    */
    public double getTurnGearRatio(){
        return Constants.turnMotorGearRatio;
    }

    /** Sets the velocity of our intake collector motors
     * @param
     */
    public void setCollectorVelocity(AngularVelocity velocity) {
    }

    /** Sets the collectors position
     * @param
     */
    public void setCollectorPosition(Angle angle) {
        turnMotor.setPosition(angle);
    }

    /** Sets the collectors turn motor velocity
     * @param
     */
    public void setCollectorTurnMotorVelocity(AngularVelocity velocity) {
        turnMotor.setControl(velocityVoltageRequest.withVelocity(velocity));
    }

    /** Checks to see if the collector is down
     * This is to help ensure that we only have one collector down at a time
     */
    public boolean isCollectorDown() {
        return false;
    }
}
