// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** This CollectorIO pretains to the back collector, it has the functions of raising
 *      lowering, and of course collecting.
 * 
 * @author Liam B
 */
public class CollectorBackIO implements CollectorIO {
    public static class Constants {
        public static final int turnMotorID = 0;
        public static final int intakeMotorLowerID = 0;
        public static final int intakeMotorUpperID = 0;
    }

    private final TalonFX turnMotor;
    private final TalonFX intakeMotorLower;
    private final TalonFX intakeMotorUpper;
    
    /**Instanciates, configures, and sets up the IO inputs for motors */
    public CollectorBackIO() {
        turnMotor = new TalonFX(Constants.turnMotorID);
        intakeMotorLower = new TalonFX(Constants.intakeMotorLowerID);
        intakeMotorUpper = new TalonFX(Constants.intakeMotorUpperID);
    }

    @Override
    /**Updates our inputs for all motors 
     * @param
    */
    public void updateInputs(CollectorIOInputs inputs) {}

    /**Sets the velocity of our collector motors
     * @param
    */
    public void setCollectorVelocity(AngularVelocity velocity) {}

    /**Puts the collector into a down position
     * @param
     */
    public void setCollectorDown(AngularVelocity velocity) {}

    /**Puts the collector into an up position
     * @param
     */
    public void setCollectorUp(AngularVelocity velocity) {}

    /**Checks to see if the collector is down
     * This is to help ensure that we only have one collector down at a time
     */
    public boolean canCollectorGoDown() {
        return false;
    }
}
