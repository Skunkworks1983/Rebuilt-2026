// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class CollectorTalonIO implements CollectorIO {
  private final TalonFX collectorMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> collectorVelocity;
  private final StatusSignal<Voltage> collectorAppliedVolts;
  private final StatusSignal<Current> collectorCurrent;
  private final StatusSignal<Current> collectorTorqueCurrent;
  private final StatusSignal<Angle> collectorPosition;

  public CollectorTalonIO() {
    collectorMotor = new TalonFX(CollectorConstants.motorID);

    var collectorConfig = new TalonFXConfiguration();

    var slot0Configs = collectorConfig.Slot0;
    slot0Configs.kP = CollectorConstants.kP;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = CollectorConstants.kD;
    slot0Configs.kS = CollectorConstants.kS;
    slot0Configs.kV = CollectorConstants.kV;
    slot0Configs.kA = CollectorConstants.kA;

    collectorConfig.MotorOutput.Inverted = CollectorConstants.motorInvert;

    tryUntilOk(5, () -> collectorMotor.getConfigurator().apply(collectorConfig, 0.25));
    collectorVelocity = collectorMotor.getVelocity();
    collectorAppliedVolts = collectorMotor.getMotorVoltage();
    collectorCurrent = collectorMotor.getStatorCurrent();
    collectorTorqueCurrent = collectorMotor.getTorqueCurrent();
    collectorPosition = collectorMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        collectorVelocity,
        collectorAppliedVolts,
        collectorCurrent,
        collectorTorqueCurrent,
        collectorPosition);

    collectorTorqueCurrent.setUpdateFrequency(250);
    ParentDevice.optimizeBusUtilizationForAll(collectorMotor);
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            collectorVelocity,
            collectorAppliedVolts,
            collectorCurrent,
            collectorTorqueCurrent,
            collectorPosition);

    inputs.connected = status.isOK();
    inputs.collectorVelocity = collectorVelocity.getValue();
    inputs.collectorVoltage = collectorAppliedVolts.getValue();
    inputs.collectorCurrent = collectorCurrent.getValue();
    inputs.collectorTorqueCurrent = collectorTorqueCurrent.getValue();
    inputs.collectorPosition = collectorPosition.getValue();
  }

  @Override
  public void intake(AngularVelocity velocity) {
    collectorMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> collectorMotor.getConfigurator().apply(configs));
  }
}
