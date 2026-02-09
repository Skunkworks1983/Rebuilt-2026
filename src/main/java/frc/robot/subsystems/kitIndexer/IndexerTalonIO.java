// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitIndexer;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class IndexerTalonIO implements IndexerIO {
  private final TalonFX intakeMotor;

  // public static class collectorConstants {
  //     public static final int intakeID = 0;
  // }

  public IndexerTalonIO() {
    intakeMotor = new TalonFX(43);

    var intakeConfig = new TalonFXConfiguration();

    var slot0Configs = intakeConfig.Slot0;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.0;
    slot0Configs.kA = 0.0;

    var intakeMotionMagicConfig = intakeConfig.MotionMagic;
    intakeMotionMagicConfig.MotionMagicCruiseVelocity = 80;
    intakeMotionMagicConfig.MotionMagicAcceleration = 160;

    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocity = intakeMotor.getVelocity().getValueAsDouble();
    inputs.indexerCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.indexerVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.indexerTorqueCurrent = intakeMotor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void intake(double dutyCycleOut) {
    intakeMotor.setControl(new DutyCycleOut(dutyCycleOut));
  }
}
