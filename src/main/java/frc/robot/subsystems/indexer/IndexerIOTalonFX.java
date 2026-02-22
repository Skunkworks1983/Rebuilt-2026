// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants.indexerConfigs;

/** This is the motor for the Indexer. it takes in a ID */
public class IndexerIOTalonFX implements IndexerIO {

  protected final TalonFX indexerMotor;

  private final Debouncer indexerDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> indexerPosition;
  public final StatusSignal<AngularVelocity> indexerAngularVelocity;
  public final StatusSignal<Voltage> indexerVoltage;
  public final StatusSignal<Current> indexerCurrent;

  public IndexerIOTalonFX(int motorID) {

    indexerMotor = new TalonFX(motorID);

    // assigns values to the statussignals
    indexerPosition = indexerMotor.getPosition();
    indexerAngularVelocity = indexerMotor.getVelocity();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getStatorCurrent();

    // creates and applies slot0 configs
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = configs.Slot0;
    slot0Configs.kP = indexerConfigs.kP;
    slot0Configs.kI = indexerConfigs.kI;
    slot0Configs.kD = indexerConfigs.kD;
    slot0Configs.kV = indexerConfigs.kV;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerMotor.getConfigurator().apply(configs);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, indexerAngularVelocity, indexerCurrent, indexerPosition, indexerVoltage);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // update the indexer inputs
    var IndexerStatus =
        BaseStatusSignal.refreshAll(
            indexerPosition, indexerAngularVelocity, indexerVoltage, indexerCurrent);

    inputs.connected = indexerDebouncer.calculate(IndexerStatus.isOK());
    inputs.position = indexerPosition.getValue();
    inputs.angularVelocity = indexerAngularVelocity.getValue();
    inputs.appliedVolts = indexerVoltage.getValue();
    inputs.currentAmps = indexerCurrent.getValue();
  }

  /** sets the control for the indexer motor to the defined velocity */
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerMotor.setControl(new VelocityVoltage(velocity).withEnableFOC(true));
  }

  /** sets the duty cycle for the indexer motor to the defined value */
  public void setIndexerDutyCycle(double dutyCycle) {
    indexerMotor.setControl(new DutyCycleOut(dutyCycle).withEnableFOC(true));
  }
}
