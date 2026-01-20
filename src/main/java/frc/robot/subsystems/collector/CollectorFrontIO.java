// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class CollectorFrontIO implements CollectorIO {
    public static class Constants {
        public static final int turnMotorID = 0;
        public static final int intakeMotorLowerID = 0;
        public static final int intakeMotorUpperID = 0;
    }

    private final TalonFX turnMotor;
    private final TalonFX intakeMotorLower;
    private final TalonFX intakeMotorUpper;
    public CollectorFrontIO() {
        turnMotor = new TalonFX(Constants.turnMotorID);
        intakeMotorLower = new TalonFX(Constants.intakeMotorLowerID);
        intakeMotorUpper = new TalonFX(Constants.intakeMotorUpperID);
    }

    @Override
    public void updateInputs(CollectorIOInputs inputs) {}

    public void setCollectorVelocity(double velocity) {}

    public void setCollectorDown(double velocity) {}

    public void setCollectorUp(double velocity) {}

    public boolean canCollectorGoDown() {
        return false;
    }
}
