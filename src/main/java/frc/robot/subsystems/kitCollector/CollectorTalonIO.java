// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitCollector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class CollectorTalonIO implements CollectorIO {
    private final TalonFX intakeMotor;

    // public static class collectorConstants {
    //     public static final int intakeID = 0;
    // } 

    public CollectorTalonIO() {
        intakeMotor = new TalonFX(0);

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

        intakeMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {
        inputs.collectorVelocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.collectorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.collectorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.collectorTorqueCurrent = intakeMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void intake(AngularVelocity velocity) {
        intakeMotor.setControl(new VelocityVoltage(velocity).withSlot(0));
    }
}
