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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** This is the motor for the hopper */
public class HopperIOTalonFX implements HopperIO {

  protected final TalonFX hopperMotor;

  private final Debouncer hopperDebouncer =
      new Debouncer(HopperBedConstants.disconnectedDebounceTimeSec, DebounceType.kFalling);

  // create status signals
  public final StatusSignal<Angle> hopperPosition;
  public final StatusSignal<AngularVelocity> hopperAngularVelocity;
  public final StatusSignal<Voltage> hopperVoltage;
  public final StatusSignal<Current> hopperCurrent;

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public HopperIOTalonFX() {

    hopperMotor = new TalonFX(HopperBedConstants.hopperMotorID, HopperBedConstants.hopperCanBus);

    // assigns values to the statussignals
    hopperPosition = hopperMotor.getPosition();
    hopperAngularVelocity = hopperMotor.getVelocity();
    hopperVoltage = hopperMotor.getMotorVoltage();
    hopperCurrent = hopperMotor.getStatorCurrent();

    // creates and applies slot0 configs
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = configs.Slot0;
    slot0Configs.kP = HopperBedConstants.hopperKP;
    slot0Configs.kD = HopperBedConstants.hopperKD;
    slot0Configs.kS = HopperBedConstants.hopperKS;
    slot0Configs.kV = HopperBedConstants.hopperKV;
    slot0Configs.kA = HopperBedConstants.hopperKA;
    configs.MotorOutput.Inverted = HopperBedConstants.hopperMotorAlignment;
    tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(configs, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, hopperAngularVelocity, hopperCurrent, hopperPosition, hopperVoltage);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // update the hopper inputs
    var HopperStatus =
        BaseStatusSignal.refreshAll(
            hopperPosition, hopperAngularVelocity, hopperVoltage, hopperCurrent);

    inputs.connected = hopperDebouncer.calculate(HopperStatus.isOK());
    inputs.position = hopperPosition.getValue();
    inputs.angularVelocity = hopperAngularVelocity.getValue();
    inputs.appliedVolts = hopperVoltage.getValue();
    inputs.currentAmps = hopperCurrent.getValue();
  }

  /** sets the control for the hopper motor to the defined velocity */
  @Override
  public void setHopperVelocity(AngularVelocity velocity) {
    hopperMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current output) {
    hopperMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setPID(Slot0Configs configs) {
    tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    hopperMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
  }
}
