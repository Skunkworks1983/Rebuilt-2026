package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CollectorModuleSimIO extends CollectorModuleTalonIO {
  private IntakeSimulation collectorSimulation;

  private DCMotorSim intakeMotorUpperSim;
  private DCMotorSim intakeMotorLowerSim;
  private DCMotorSim turnMotorSim;

  final double minimumCollectorRadiansToCollect = 0.1;
  final double minimumCollectorRadiansToBeDown = 1.55;
  final double minimumCollectorSpeedToCollect = 5;

  public static enum CollectorSide {
    Left,
    Right
  }

  public final CollectorSide collectorSide;

  public CollectorModuleSimIO(
      AbstractDriveTrainSimulation driveTrain,
      CollectorSide collectorSide,
      CollectorConstants constants) {

    super(constants);

    this.collectorSide = collectorSide;
    // collectorSide, 0 = left, 1 = right
    switch (collectorSide) {
      case Left:
        this.collectorSimulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrain,
                Meters.of(0.7),
                Meters.of(0.2),
                IntakeSimulation.IntakeSide.LEFT,
                1);
        break;
      case Right:
        this.collectorSimulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrain,
                Meters.of(0.7),
                Meters.of(0.2),
                IntakeSimulation.IntakeSide.RIGHT,
                1);
        break;
    }

    LinearSystem linearSystemIntakeMotors = LinearSystemId.createDCMotorSystem(0.002, 0.02);
    LinearSystem linearSystemTurnMotor = LinearSystemId.createDCMotorSystem(1, 1);
    double[] stdevs = {0.001, 0.001};

    intakeMotorLowerSim = new DCMotorSim(linearSystemIntakeMotors, DCMotor.getKrakenX60(1), stdevs);
    intakeMotorUpperSim = new DCMotorSim(linearSystemIntakeMotors, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim = new DCMotorSim(linearSystemTurnMotor, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim.setAngle(0);
  }

  /**
   * detects whether fuel is inside collector
   *
   * @return
   */
  public boolean isFuelInsideCollector() {
    return collectorSimulation.getGamePiecesAmount() != 0;
  }

  double j;

  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  @Override
  public void updateInputs(CollectorIOInputs input) {

    var turnMotorTalonFX = super.turnMotor.getSimState();
    var intakeMotorLowerTalonFX = super.intakeMotorLower.getSimState();
    var intakeMotorUpperTalonFX = super.intakeMotorUpper.getSimState();

    turnMotorTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());
    intakeMotorLowerTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());
    intakeMotorUpperTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update simulations and input voltage for motors
    turnMotorSim.setInputVoltage(turnMotorTalonFX.getMotorVoltage());
    turnMotorSim.update(0.02);

    intakeMotorLowerSim.setInputVoltage(intakeMotorLowerTalonFX.getMotorVoltage());
    intakeMotorLowerSim.update(0.02);

    intakeMotorUpperSim.setInputVoltage(intakeMotorUpperTalonFX.getMotorVoltage());
    intakeMotorUpperSim.update(0.02);

    turnMotorTalonFX.setRotorVelocity(turnMotorSim.getAngularVelocity());
    intakeMotorLowerTalonFX.setRotorVelocity(intakeMotorLowerSim.getAngularVelocity());
    intakeMotorUpperTalonFX.setRotorVelocity(intakeMotorUpperSim.getAngularVelocity());

    turnMotorTalonFX.setRawRotorPosition(turnMotorSim.getAngularPosition());

    // Turn Motor Inputs
    input.turnConnected = true;
    input.turnPosition = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad());
    input.turnVelocity = turnMotorSim.getAngularVelocity();
    input.turnAppliedVolts = turnMotorTalonFX.getMotorVoltageMeasure();
    input.turnCurrentAmps = Current.ofBaseUnits(turnMotorSim.getCurrentDrawAmps(), Amps);

    // Lower Intake Motor Inputs
    input.lowerConnected = true;
    input.lowerPositionRad = intakeMotorLowerSim.getAngularPosition();
    input.lowerVelocity = intakeMotorLowerSim.getAngularVelocity();
    input.lowerAppliedVolts = intakeMotorLowerTalonFX.getMotorVoltageMeasure();
    input.lowerCurrentAmps = Current.ofBaseUnits(intakeMotorLowerSim.getCurrentDrawAmps(), Amps);

    // Upper Intake Motor Inputs
    input.upperConnected = true;
    input.upperPositionRad = intakeMotorUpperSim.getAngularPosition();
    input.upperVelocity = intakeMotorUpperSim.getAngularVelocity();
    input.upperAppliedVolts = intakeMotorUpperTalonFX.getMotorVoltageMeasure();
    input.upperCurrentAmps = Current.ofBaseUnits(intakeMotorUpperSim.getCurrentDrawAmps(), Amps);

    if (turnMotorSim.getAngularPositionRad() < minimumCollectorRadiansToCollect
        && intakeMotorLowerSim.getAngularVelocityRadPerSec() > minimumCollectorSpeedToCollect) {

      collectorSimulation.startIntake();
    } else {
      collectorSimulation.stopIntake();
    }
  }
}
