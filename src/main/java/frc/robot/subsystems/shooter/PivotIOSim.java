package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim extends PivotIOTalonFX {
    LinearSystem linearSystem = LinearSystemId.createDCMotorSystem(0.5, 0.2);
    private DCMotorSim pivotSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX60(1), 0.002, 0.001);

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        TalonFXSimState pivotTalonFX = super.pivotMotor.getSimState();
        pivotTalonFX.Orientation = ChassisReference.Clockwise_Positive;

        pivotTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());

        pivotSim.setInputVoltage(pivotTalonFX.getMotorVoltage());
        pivotSim.update(0.02);


        pivotTalonFX.setRawRotorPosition(pivotSim.getAngularPosition());
        pivotTalonFX.setRotorVelocity(pivotSim.getAngularVelocity());

        inputs.connected = true;
        inputs.pivotPosition = pivotSim.getAngularPosition();
        inputs.pivotStatorCurrentAmps = pivotTalonFX.getSupplyCurrentMeasure();
        inputs.pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrentMeasure();
    }

}
