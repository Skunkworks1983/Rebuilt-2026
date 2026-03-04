// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** This is the motor for the Indexer. it takes in a ID */
public class IndexerIOTalonFX implements IndexerIO {

  protected final TalonFX indexerMotor;

  private final Debouncer indexerDebouncer =
      new Debouncer(IndexerConstants.disconnectedDebounceTimeSec, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> indexerPosition;
  public final StatusSignal<AngularVelocity> indexerAngularVelocity;
  public final StatusSignal<Voltage> indexerVoltage;
  public final StatusSignal<Current> indexerCurrent;

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public IndexerIOTalonFX(int motorID) {

    indexerMotor = new TalonFX(motorID, IndexerConstants.indexerCanBus);

    // assigns values to the statussignals
    indexerPosition = indexerMotor.getPosition();
    indexerAngularVelocity = indexerMotor.getVelocity();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getStatorCurrent();

    // creates and applies slot0 configs
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = configs.Slot0;
    slot0Configs.kP = IndexerConstants.indexerKP;
    slot0Configs.kD = IndexerConstants.indexerKD;
    slot0Configs.kS = IndexerConstants.indexerKS;
    slot0Configs.kV = IndexerConstants.indexerKV;
    configs.MotorOutput.Inverted = IndexerConstants.indexerMotorAlignment;

    tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(configs, 0.25));

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
  @Override
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerMotor.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current output) {
    indexerMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    indexerMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
  }
}
