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
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorModuleConstants.Side;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.ctre.phoenix6.sim.ChassisReference;

public class CollectorModuleSimIO extends CollectorModuleTalonIO {
  private IntakeSimulation collectorSimulation;

  private DCMotorSim intakeMotorSim;
  private DCMotorSim turnMotorSim;

  private DIOSim upperLimitSim;
  private DIOSim lowerLimitSim;

  public Side side;

  private final double minimumCollectorRadiansToCollect = 1.55;
  final double minimumCollectorSpeedToCollect = 5;

  public CollectorModuleSimIO(
      AbstractDriveTrainSimulation driveTrain, CollectorModuleConstants constants) {

    super(constants);

    this.side = constants.collectorSide;

    switch (side) {
      case LEFT:
        this.collectorSimulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveTrain,
                Meters.of(0.7),
                Meters.of(0.2),
                IntakeSimulation.IntakeSide.LEFT,
                1);
        break;
      case RIGHT:
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

    intakeMotorSim = new DCMotorSim(linearSystemIntakeMotors, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim = new DCMotorSim(linearSystemTurnMotor, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim.setAngle(0);

    upperLimitSim = new DIOSim(super.upperLimitSwitch);
    lowerLimitSim = new DIOSim(super.lowerLimitSwitch);

    upperLimitSim.setValue(true);
    lowerLimitSim.setValue(false);
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

  /** Change upper limit switch value */
  public boolean changeUpperLimitValue() {
    return turnMotorSim
        .getAngularPosition()
        .lte(constants.CollectorUpPos.plus(constants.CollectorPosTolerance));
  }

  /** Change lower limit switch value */
  public boolean changeLowerLimitValue() {
    return turnMotorSim
        .getAngularPosition()
        .gte(constants.CollectorDownPos.plus(constants.CollectorPosTolerance));
  }

  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  @Override
  public void updateInputs(CollectorIOInputs input) {

    var turnMotorTalonFX = super.turnMotor.getSimState();
    var intakeMotorTalonFX = super.intakeMotor.getSimState();

    turnMotorTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());
    intakeMotorTalonFX.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update simulations and input voltage for motors
    turnMotorSim.setInputVoltage(turnMotorTalonFX.getMotorVoltage());
    turnMotorSim.update(0.02);

    intakeMotorSim.setInputVoltage(intakeMotorTalonFX.getMotorVoltage());
    intakeMotorSim.update(0.02);

    turnMotorTalonFX.setRotorVelocity(turnMotorSim.getAngularVelocity());
    intakeMotorTalonFX.setRotorVelocity(intakeMotorSim.getAngularVelocity());

    turnMotorTalonFX.setRawRotorPosition(turnMotorSim.getAngularPosition());

    // Turn Motor Inputs
    input.turnConnected = true;
    input.turnPosition = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad());
    input.turnVelocity = turnMotorSim.getAngularVelocity();
    input.turnAppliedVolts = turnMotorTalonFX.getMotorVoltageMeasure();
    input.turnCurrentAmps = Current.ofBaseUnits(turnMotorSim.getCurrentDrawAmps(), Amps);

    // Lower Intake Motor Inputs
    input.intakeConnected = true;
    input.intakePositionRad = intakeMotorSim.getAngularPosition();
    input.intakeVelocity = intakeMotorSim.getAngularVelocity();
    input.intakeAppliedVolts = intakeMotorTalonFX.getMotorVoltageMeasure();
    input.intakeCurrentAmps = Current.ofBaseUnits(intakeMotorSim.getCurrentDrawAmps(), Amps);

    // limit switches inputs
    upperLimitSim.setValue(changeUpperLimitValue());
    lowerLimitSim.setValue(changeLowerLimitValue());
    input.upperTriggered = upperLimitSim.getValue();
    input.lowerTriggered = lowerLimitSim.getValue();

    if (turnMotorSim.getAngularPositionRad() > minimumCollectorRadiansToCollect
        && intakeMotorSim.getAngularVelocityRadPerSec() > minimumCollectorSpeedToCollect) {
      collectorSimulation.startIntake();
    } else {
      collectorSimulation.stopIntake();
    }
  }
}
