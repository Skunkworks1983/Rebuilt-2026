// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** This is the motor for the Indexer. it takes in a ID */
public class IndexerIOTalonFX implements IndexerIO {

  protected final TalonFX primaryIndexerMotor;
  protected final TalonFX followerIndexerMotor;

  private final Debouncer indexerDebouncer =
      new Debouncer(IndexerConstants.disconnectedDebounceTimeSec, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> primaryPosition;
  public final StatusSignal<AngularVelocity> primaryVelocity;
  public final StatusSignal<Voltage> primaryVoltage;
  public final StatusSignal<Current> primaryCurrent;

  public final StatusSignal<Angle> followerPosition;
  public final StatusSignal<AngularVelocity> followerVelocity;
  public final StatusSignal<Voltage> followerVoltage;
  public final StatusSignal<Current> followerCurrent;

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public IndexerIOTalonFX() {
    primaryIndexerMotor =
        new TalonFX(IndexerConstants.leftIndexerMotorID, IndexerConstants.indexerCanBus);
    followerIndexerMotor =
        new TalonFX(IndexerConstants.rightIndexerMotorID, IndexerConstants.indexerCanBus);

    TalonFXConfiguration primaryConfig = new TalonFXConfiguration();
    primaryConfig.Feedback.SensorToMechanismRatio = IndexerConstants.indexerGearRatio;
    primaryConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.indexerSupplyCurrentLimitAmps;
    primaryConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.indexerStatorCurrentLimitAmps;

    Slot0Configs slot0Configs = primaryConfig.Slot0;
    slot0Configs.kP = IndexerConstants.indexerKP;
    slot0Configs.kD = IndexerConstants.indexerKD;
    slot0Configs.kS = IndexerConstants.indexerKS;
    slot0Configs.kV = IndexerConstants.indexerKV;
    primaryConfig.MotorOutput.Inverted = IndexerConstants.primaryMotorInvert;

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.Feedback.SensorToMechanismRatio = IndexerConstants.indexerGearRatio;
    followerConfig.CurrentLimits.SupplyCurrentLimit =
        IndexerConstants.indexerSupplyCurrentLimitAmps;
    followerConfig.CurrentLimits.StatorCurrentLimit =
        IndexerConstants.indexerStatorCurrentLimitAmps;

    tryUntilOk(5, () -> primaryIndexerMotor.getConfigurator().apply(primaryConfig, 0.25));
    tryUntilOk(5, () -> followerIndexerMotor.getConfigurator().apply(followerConfig, 0.25));
    primaryPosition = primaryIndexerMotor.getPosition();
    primaryVelocity = primaryIndexerMotor.getVelocity();
    primaryVoltage = primaryIndexerMotor.getMotorVoltage();
    primaryCurrent = primaryIndexerMotor.getStatorCurrent();

    followerPosition = followerIndexerMotor.getPosition();
    followerVelocity = followerIndexerMotor.getVelocity();
    followerVoltage = followerIndexerMotor.getMotorVoltage();
    followerCurrent = followerIndexerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        primaryVelocity,
        primaryCurrent,
        primaryPosition,
        primaryVoltage,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerCurrent);

    ParentDevice.optimizeBusUtilizationForAll(primaryIndexerMotor, followerIndexerMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // update the indexer inputs
    var IndexerStatus =
        BaseStatusSignal.refreshAll(
            primaryPosition, primaryVelocity, primaryVoltage, primaryCurrent);

    inputs.connected = indexerDebouncer.calculate(IndexerStatus.isOK());
    inputs.position = primaryPosition.getValue();
    inputs.angularVelocity = primaryVelocity.getValue();
    inputs.appliedVolts = primaryVoltage.getValue();
    inputs.currentAmps = primaryCurrent.getValue();
  }

  /** sets the control for the indexer motor to the defined velocity */
  @Override
  public void setIndexerVelocity(AngularVelocity velocity) {
    primaryIndexerMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> primaryIndexerMotor.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    primaryIndexerMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
  }
}
