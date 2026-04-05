// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** This is the motor for the Trigger. it takes in a ID */
public class TriggerIOTalonFX implements TriggerIO {

  protected final TalonFX primaryTriggerMotor;
  protected final TalonFX followerTriggerMotor;

  private final Debouncer triggerDebouncer =
      new Debouncer(TriggerConstants.disconnectedDebounceTimeSec, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> primaryPosition;
  public final StatusSignal<AngularVelocity> primaryVelocity;
  public final StatusSignal<Voltage> primaryVoltage;
  public final StatusSignal<Current> primaryCurrent;
  public final StatusSignal<Double> primaryClosedLoopError;

  public final StatusSignal<Angle> followerPosition;
  public final StatusSignal<AngularVelocity> followerVelocity;
  public final StatusSignal<Voltage> followerVoltage;
  public final StatusSignal<Current> followerCurrent;
  public final StatusSignal<Double> followerClosedLoopError;

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public TriggerIOTalonFX() {
    primaryTriggerMotor =
        new TalonFX(TriggerConstants.leftTriggerMotorID, TriggerConstants.triggerCanBus);
    followerTriggerMotor =
        new TalonFX(TriggerConstants.rightTriggerMotorID, TriggerConstants.triggerCanBus);

    TalonFXConfiguration primaryConfig = new TalonFXConfiguration();
    primaryConfig.Feedback.SensorToMechanismRatio = TriggerConstants.triggerGearRatio;
    primaryConfig.CurrentLimits.SupplyCurrentLimit = TriggerConstants.triggerSupplyCurrentLimitAmps;
    primaryConfig.CurrentLimits.StatorCurrentLimit = TriggerConstants.triggerStatorCurrentLimitAmps;
    primaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    primaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    Slot0Configs slot0Configs = primaryConfig.Slot0;
    slot0Configs.kP = TriggerConstants.triggerKP;
    slot0Configs.kD = TriggerConstants.triggerKD;
    slot0Configs.kS = TriggerConstants.triggerKS;
    slot0Configs.kV = TriggerConstants.triggerKV;
    primaryConfig.MotorOutput.Inverted = TriggerConstants.primaryMotorInvert;

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.Feedback.SensorToMechanismRatio = TriggerConstants.triggerGearRatio;
    followerConfig.CurrentLimits.SupplyCurrentLimit =
        TriggerConstants.triggerSupplyCurrentLimitAmps;
    followerConfig.CurrentLimits.StatorCurrentLimit =
        TriggerConstants.triggerStatorCurrentLimitAmps;
    tryUntilOk(5, () -> primaryTriggerMotor.getConfigurator().apply(primaryConfig, 0.25));
    tryUntilOk(5, () -> followerTriggerMotor.getConfigurator().apply(followerConfig, 0.25));
    followerTriggerMotor.setControl(
        new Follower(primaryTriggerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    primaryPosition = primaryTriggerMotor.getPosition();
    primaryVelocity = primaryTriggerMotor.getVelocity();
    primaryVoltage = primaryTriggerMotor.getMotorVoltage();
    primaryCurrent = primaryTriggerMotor.getStatorCurrent();
    primaryClosedLoopError = primaryTriggerMotor.getClosedLoopError();

    followerPosition = followerTriggerMotor.getPosition();
    followerVelocity = followerTriggerMotor.getVelocity();
    followerVoltage = followerTriggerMotor.getMotorVoltage();
    followerCurrent = followerTriggerMotor.getStatorCurrent();
    followerClosedLoopError = followerTriggerMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        primaryVelocity,
        primaryCurrent,
        primaryPosition,
        primaryVoltage,
        primaryClosedLoopError,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerCurrent,
        followerClosedLoopError);

    ParentDevice.optimizeBusUtilizationForAll(primaryTriggerMotor, followerTriggerMotor);
  }

  @Override
  public void updateInputs(TriggerIOInputs inputs) {
    // update the trigger inputs
    var primaryTriggerStatus =
        BaseStatusSignal.refreshAll(
            primaryPosition,
            primaryVelocity,
            primaryVoltage,
            primaryCurrent,
            primaryClosedLoopError);

    var followerTriggerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition,
            followerVelocity,
            followerVoltage,
            followerCurrent,
            followerClosedLoopError);

    inputs.primaryConnected = triggerDebouncer.calculate(primaryTriggerStatus.isOK());
    inputs.primaryPosition = primaryPosition.getValue();
    inputs.primaryAngularVelocity = primaryVelocity.getValue();
    inputs.primaryAppliedVolts = primaryVoltage.getValue();
    inputs.primaryCurrentAmps = primaryCurrent.getValue();
    inputs.primaryClosedLoopError = RotationsPerSecond.of(primaryClosedLoopError.getValue());

    inputs.followerConnected = triggerDebouncer.calculate(followerTriggerStatus.isOK());
    inputs.followerPosition = followerPosition.getValue();
    inputs.followerAngularVelocity = followerVelocity.getValue();
    inputs.followerAppliedVolts = followerVoltage.getValue();
    inputs.followerCurrentAmps = followerCurrent.getValue();
    inputs.followerClosedLoopError = RotationsPerSecond.of(followerClosedLoopError.getValue());
  }

  /** sets the control for the trigger motor to the defined velocity */
  @Override
  public void setTriggerVelocity(AngularVelocity velocity) {
    primaryTriggerMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> primaryTriggerMotor.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    primaryTriggerMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
  }
}
