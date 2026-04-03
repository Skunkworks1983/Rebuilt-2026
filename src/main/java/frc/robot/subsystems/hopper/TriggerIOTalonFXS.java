// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** This is the motor for the Trigger. it takes in a ID */
public class TriggerIOTalonFXS implements TriggerIO {

  protected final TalonFXS triggerMotor;

  private final Debouncer triggerDebouncer =
      new Debouncer(TriggerConstants.disconnectedDebounceTimeSec, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> triggerPosition;
  public final StatusSignal<AngularVelocity> triggerAngularVelocity;
  public final StatusSignal<Voltage> triggerVoltage;
  public final StatusSignal<Current> triggerCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public TriggerIOTalonFXS(int motorID) {

    triggerMotor = new TalonFXS(motorID, TriggerConstants.triggerCanBus);

    // assigns values to the statussignals
    triggerPosition = triggerMotor.getPosition();
    triggerAngularVelocity = triggerMotor.getVelocity();
    triggerVoltage = triggerMotor.getMotorVoltage();
    triggerCurrent = triggerMotor.getStatorCurrent();

    // creates and applies slot0 configs
    TalonFXSConfiguration configs = new TalonFXSConfiguration();
    configs.ExternalFeedback.SensorToMechanismRatio =
        TriggerConstants.triggerSensorToMechanismRatio;
    Slot0Configs slot0Configs = configs.Slot0;
    slot0Configs.kP = TriggerConstants.triggerKP;
    slot0Configs.kD = TriggerConstants.triggerKD;
    slot0Configs.kS = TriggerConstants.triggerKS;
    slot0Configs.kV = TriggerConstants.triggerKV;
    configs.MotorOutput.Inverted = TriggerConstants.triggerMotorAlignment;
    configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    tryUntilOk(5, () -> triggerMotor.getConfigurator().apply(configs, 0.25));
    slot0Configs.kP = TriggerConstants.triggerKP;
    slot0Configs.kD = TriggerConstants.triggerKD;
    slot0Configs.kS = TriggerConstants.triggerKS;
    slot0Configs.kV = TriggerConstants.triggerKV;
    configs.MotorOutput.Inverted = TriggerConstants.triggerMotorAlignment;

    tryUntilOk(5, () -> triggerMotor.getConfigurator().apply(configs, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, triggerAngularVelocity, triggerCurrent, triggerPosition, triggerVoltage);
  }

  @Override
  public void updateInputs(TriggerIOInputs inputs) {
    // update the trigger inputs
    var TriggerStatus =
        BaseStatusSignal.refreshAll(
            triggerPosition, triggerAngularVelocity, triggerVoltage, triggerCurrent);

    inputs.connected = triggerDebouncer.calculate(TriggerStatus.isOK());
    inputs.position = triggerPosition.getValue();
    inputs.angularVelocity = triggerAngularVelocity.getValue();
    inputs.appliedVolts = triggerVoltage.getValue();
    inputs.currentAmps = triggerCurrent.getValue();
  }

  /** sets the control for the trigger motor to the defined velocity */
  @Override
  public void setTriggerVelocity(AngularVelocity velocity) {
    triggerMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current output) {
    triggerMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> triggerMotor.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    triggerMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
  }

  @Override
  public void setVoltageOut(Voltage voltage) {
    triggerMotor.setControl(new VoltageOut(voltage));
  }
}
