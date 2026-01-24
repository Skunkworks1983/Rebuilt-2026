// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This is the collector Subsystem
 *   <ul>
 *      <li>It takes in two collector module IO's to assign to two collector modules</li>
 *   </ul>
 */
public class Collector extends SubsystemBase{

    private CollectorModule[] collectors;
    private int downCollector;

    /**
     * Collector subsystem.
     *
     * <ul>
     *   <li>Takes in two collector module IOs and assigns them to the collector modules.</li>
     * </ul>
     *
     * <p>
     * <img src="https://media.giphy.com/media/jVFgWDtkY5v2M/giphy.gif"
     *      alt="Our Collector 💕">
     * </p>
     *
     * @param leftIO  left collector
     * @param rightIO right collector
     */
    public Collector(CollectorModuleIO leftIO, CollectorModuleIO rightIO) {
        collectors[0] = new CollectorModule(leftIO);
        collectors[1] = new CollectorModule(rightIO);

        downCollector = -2; // -1 == neither, 0 == left collector mod, 1 == right collector mod, // 2 = both are down!
    }

    /** @return -Which collector is down */
    public String getDownCollector() {
        switch (downCollector) {
            case -1: 
                return "NEITHER";

            case 0:
                return "LEFT";

            case 1:
                return "RIGHT";

            case 2:
                return "BOTH";

            default:
                if(collectors[0].getPosition().in(Degrees) < 0.0 && collectors[1].getPosition().in(Degrees) < 0.0) {// TODO: replace 0.0 w/ collector up position
                    return "BOTH!!!"; // if this is returned and we didnt intend for it, it is an EMERGENCY!!
                } else if(collectors[0].getPosition().in(Degrees) < 0.0){
                    return "LEFT";
                } else if(collectors[1].getPosition().in(Degrees) < 0.0){
                    return "RIGHT";
                } else {
                    return "NEITHER";
                }
        }
    }

    /** Sets a variable to tell us which collector is down
     * @param downed -collector is now down
     */
    public void setDownCollector(String downed) {
        switch (downed) {
            case "NEITHER": 
                downCollector = -1;
                collectors[0].setIsCollectorDown(false);
                collectors[1].setIsCollectorDown(false);
                break;

            case "LEFT":
                downCollector = 0;
                collectors[0].setIsCollectorDown(true);
                break;
        
            case "RIGHT":
                downCollector = 1;
                collectors[1].setIsCollectorDown(true);
                break;

            case "BOTH": 
                downCollector = 2;
                collectors[0].setIsCollectorDown(true);
                collectors[0].setIsCollectorDown(true);
                break;

            default:
                break;
        }
    }

    /**
     * @return -Left collector Module
     */
    public CollectorModule getLeftCollector() {
        return collectors[0];
    }

    /**
     * @return -Right collector module
     */
    public CollectorModule getRightCollector() {
        return collectors[1];
    }
}
