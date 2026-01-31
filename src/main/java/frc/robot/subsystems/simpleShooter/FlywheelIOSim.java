package frc.robot.subsystems.simpleShooter;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private final FlywheelSim flywheelSim;
  private final FlywheelSim secondaryFlywheelSim;

  public FlywheelIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.flywheelInertia.magnitude(),
                ShooterConstants.flywheelGearRatio),
            DCMotor.getKrakenX60Foc(1));
    secondaryFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.flywheelInertia.magnitude(),
                ShooterConstants.flywheelGearRatio),
            DCMotor.getKrakenX60Foc(1));

    super.flywheelMotor.getConfigurator().apply(ShooterConstants.simFlywheelConfigs);
    super.flywheelFollower.getConfigurator().apply(ShooterConstants.simFlywheelConfigs);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var talonFXSim = super.flywheelMotor.getSimState();
    var talonFXSimSecondary = super.flywheelFollower.getSimState();

    talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
    talonFXSimSecondary.Orientation = ChassisReference.Clockwise_Positive;

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    talonFXSimSecondary.setSupplyVoltage(RobotController.getBatteryVoltage());

    Voltage primaryAppliedVolts = talonFXSim.getMotorVoltageMeasure();
    Voltage secondayAppliedVolts = talonFXSimSecondary.getMotorVoltageMeasure();

    flywheelSim.setInputVoltage(primaryAppliedVolts.magnitude());
    secondaryFlywheelSim.setInputVoltage(secondayAppliedVolts.magnitude());

    flywheelSim.update(0.020);
    secondaryFlywheelSim.update(0.020);

    talonFXSim.setRotorVelocity(
        flywheelSim.getAngularVelocity().times(ShooterConstants.flywheelGearRatio));
    talonFXSimSecondary.setRotorVelocity(
        secondaryFlywheelSim.getAngularVelocity().times(ShooterConstants.flywheelGearRatio));

    inputs.primaryConnected = true;
    inputs.followerConnected = true;

    inputs.primaryVelocity = flywheelSim.getAngularVelocity();
    inputs.followerVelocity = secondaryFlywheelSim.getAngularVelocity();

    inputs.primaryAppliedVolts = primaryAppliedVolts;
    inputs.followerAppliedVolts = secondayAppliedVolts;

    inputs.primaryStatorCurrentAmps = talonFXSim.getSupplyCurrentMeasure();
    inputs.followerStatorCurrentAmps = talonFXSimSecondary.getSupplyCurrentMeasure();

    inputs.primaryTorqueCurrentAmps = talonFXSim.getTorqueCurrentMeasure();
    inputs.followerTorqueCurrentAmps = talonFXSimSecondary.getTorqueCurrentMeasure();
  }
}
