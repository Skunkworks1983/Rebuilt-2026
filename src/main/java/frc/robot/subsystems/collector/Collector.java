package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

    public Collector(CollectorIO s) {
        collectorIO = s;
    }

    private CollectorIO collectorIO;

}
