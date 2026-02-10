// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitIndexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerTalonIO implements IndexerIO {
  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrent;
  private final StatusSignal<Current> indexerTorqueCurrent;
  private final StatusSignal<Angle> indexerPosition;

  public IndexerTalonIO() {
    intakeMotor = new TalonFX(IndexerConstants.motorID, "rio");

    var intakeConfig = new TalonFXConfiguration();

    var slot0Configs = intakeConfig.Slot0;
    slot0Configs.kP = IndexerConstants.kP;
    slot0Configs.kD = IndexerConstants.kD;
    slot0Configs.kS = IndexerConstants.kS;
    slot0Configs.kV = IndexerConstants.kV;
    slot0Configs.kA = IndexerConstants.kA;

    intakeConfig.MotorOutput.Inverted = IndexerConstants.motorInvert;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    indexerVelocity = intakeMotor.getVelocity();
    indexerAppliedVolts = intakeMotor.getMotorVoltage();
    indexerCurrent = intakeMotor.getStatorCurrent();
    indexerTorqueCurrent = intakeMotor.getTorqueCurrent();
    indexerPosition = intakeMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, indexerVelocity, indexerAppliedVolts, indexerCurrent, indexerPosition);

    indexerTorqueCurrent.setUpdateFrequency(250);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            indexerVelocity,
            indexerAppliedVolts,
            indexerCurrent,
            indexerTorqueCurrent,
            indexerPosition);

    inputs.connected = status.isOK();
    inputs.indexerVelocity = indexerVelocity.getValue();
    inputs.indexerVoltage = indexerAppliedVolts.getValue();
    inputs.indexerCurrent = indexerCurrent.getValue();
    inputs.indexerTorqueCurrent = indexerTorqueCurrent.getValue();
    inputs.indexerPosition = indexerPosition.getValue();
  }

  @Override
  public void intake(AngularVelocity velocity) {
    intakeMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPID(Slot0Configs gains) {
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(gains));
  }
}
