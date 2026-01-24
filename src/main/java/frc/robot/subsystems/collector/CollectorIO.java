package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface CollectorIO {

    @AutoLog
    public class CollectorIOInputs {
        public boolean extended = false;
        public AngularVelocity speed;
        public int amountOfFuelInside;
    }

    public void setRunning(boolean runIntake);

    public boolean isFuelInsideCollector();

    public void updateInputs(CollectorIOInputs input);
}
