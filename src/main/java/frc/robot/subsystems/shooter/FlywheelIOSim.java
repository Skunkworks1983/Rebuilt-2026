package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private final FlywheelSim flywheelSim;

  public FlywheelIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.flywheelInertia.magnitude(),
                ShooterConstants.flywheelGearRatio),
            DCMotor.getKrakenX60Foc(1));

    super.primaryFlywheelMotor.getConfigurator().apply(ShooterConstants.simFlywheelConfigs);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var talonFXSim = super.primaryFlywheelMotor.getSimState();

    talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    Voltage primaryAppliedVolts = talonFXSim.getMotorVoltageMeasure();
    flywheelSim.setInputVoltage(primaryAppliedVolts.magnitude());

    flywheelSim.update(0.020);

    talonFXSim.setRotorVelocity(
        flywheelSim.getAngularVelocity().times(ShooterConstants.flywheelGearRatio));

    inputs.primaryConnected = true;
    inputs.primaryAngularVelocity =
        flywheelSim.getAngularVelocity().times(ShooterConstants.flywheelGearRatio);
    inputs.primaryAppliedVolts = primaryAppliedVolts;
    inputs.primaryStatorCurrentAmps = talonFXSim.getSupplyCurrentMeasure();
    inputs.primaryTorqueCurrentAmps = talonFXSim.getTorqueCurrentMeasure();
  }

  // public void setVelocity(AngularVelocity velocity) {
  //   flywheelSim.setAngularVelocity(velocity.in(RadiansPerSecond));
  // }
}
