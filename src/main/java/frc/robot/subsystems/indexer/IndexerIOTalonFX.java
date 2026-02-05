// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IndexerConstants.indexerConfigs;

/**
 * This is the motor for the Indexer. it takes in a ID
*/
public class IndexerIOTalonFX implements IndexerIO {

  VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  private final TalonFX indexerMotor;

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
    indexerMotor.getConfigurator().apply(configs);
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

  /** 
  * sets the control for the indexer motor to the defined velocity
  */
  public void setIndexerVelocity(AngularVelocity VelocityRotationsPerSec) {
    indexerMotor.setControl(
        velocityVoltageRequest.withVelocity(VelocityRotationsPerSec.in(RotationsPerSecond)));
  }
}
