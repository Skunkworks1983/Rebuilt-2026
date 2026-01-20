// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

/** Add your docs here. */
public class CollectorFrontIO implements CollectorIO {
    public static class Constants {}

    @Override
    public void updateInputs(CollectorIOInputs inputs) {}

    public void setCollectorVelocity(double velocity) {}

    public void setCollectorDown(double velocity) {}

    public void setCollectorUp(double velocity) {}

    public boolean canCollectorGoDown() {
        return false;
    }
}
