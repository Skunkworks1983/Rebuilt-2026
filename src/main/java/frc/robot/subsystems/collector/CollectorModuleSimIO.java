package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CollectorModuleSimIO extends CollectorModuleTalonIO {
  public IntakeSimulation collectorSimulation;

  private DCMotorSim intakeMotorSim;
  private DCMotorSim turnMotorSim;

  private DIOSim upperLimitSim;
  private DIOSim lowerLimitSim;

  private int fuelInCollectorSim = 0;

  private final double minimumCollectorRadiansToCollect = 1.55;
  final double minimumCollectorSpeedToCollect = 5;

  public CollectorModuleSimIO(AbstractDriveTrainSimulation driveTrain) {

    this.collectorSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.FRONT,
            1);

    // SimulatedArena.getInstance().addIntakeSimulation(collectorSimulation);

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
        .gte(constants.CollectorDownPosDefault.plus(constants.CollectorPosTolerance));
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

    double turnGearRatio = super.constants.turnMotorGearRatio;

    turnMotorTalonFX.setRawRotorPosition(
        (turnMotorSim.getAngularPositionRad() / (2 * Math.PI)) * turnGearRatio);
    turnMotorTalonFX.setRotorVelocity(
        (turnMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI)) * turnGearRatio);

    intakeMotorTalonFX.setRotorVelocity(turnMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI));

    // Turn Motor Inputs
    input.turnConnected = true; //
    input.turnPosition = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad()); //
    input.turnVelocity = turnMotorSim.getAngularVelocity(); //
    input.turnAppliedVolts = turnMotorTalonFX.getMotorVoltageMeasure(); //
    input.turnCurrentAmps = Current.ofBaseUnits(turnMotorSim.getCurrentDrawAmps(), Amps); //
    input.turnClosedLoopError = 0.0;
    input.turnSetPoint = Radians.of(0);

    // Intake Motor Inputs for lower
    input.intakeConnected = true; //
    input.intakePositionRad = intakeMotorSim.getAngularPosition(); //
    input.intakeVelocity = intakeMotorSim.getAngularVelocity(); //
    input.intakeVelocitySetPoint = DegreesPerSecond.of(0.0);
    input.intakeAppliedVolts = intakeMotorTalonFX.getMotorVoltageMeasure(); //
    input.intakeCurrentAmps = Current.ofBaseUnits(intakeMotorSim.getCurrentDrawAmps(), Amps); //

    // Magnet Switch Inputs
    input.upperTriggered = upperLimitSim.getValue();
    input.lowerTriggered = lowerLimitSim.getValue();

    // When were on our way down
    input.isUp = isCollectorUp();
    input.isDown = isCollectorDown();

    if (Math.abs(turnMotorSim.getAngularPositionRad()) > minimumCollectorRadiansToCollect
        && intakeMotorSim.getAngularVelocityRadPerSec() > minimumCollectorSpeedToCollect
        && fuelInCollectorSim <= 25) {
      collectorSimulation.startIntake();
    } else {
      collectorSimulation.stopIntake();
    }

    if (collectorSimulation.getGamePiecesAmount() > 0) {
      int fuelToAdd = collectorSimulation.getGamePiecesAmount();
      fuelInCollectorSim += fuelToAdd;
      collectorSimulation.setGamePiecesCount(0);
    }
  }

  /**
   * Checks to see if the collector is down
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   *
   * @return whether or not collector is down
   */
  public boolean isCollectorDown() {
    return !lowerLimitSim.getValue()
        || turnMotorSim
            .getAngularPosition()
            .gte(
                Radians.of(constants.CollectorDownPos.get())
                    .minus(constants.CollectorPosTolerance));
  }

  /**
   * Checks to see if the collector is up
   *
   * <ul>
   *   <li>This is to help ensure that we only have one collectordown at a time
   * </ul>
   *
   * @return whether or not collector is up
   */
  public boolean isCollectorUp() {
    return !lowerLimitSim.getValue()
        || turnMotorSim
            .getAngularPosition()
            .lte(constants.CollectorUpPos.plus(constants.CollectorPosTolerance));
  }

  public Angle getPosition() {
    return turnMotorSim.getAngularPosition();
  }

  public void removeSimFuel() {
    collectorSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());
  }

  public int getCollectedSimFuel() {
    return fuelInCollectorSim;
  }

  public void removeCollectedSimFuel() {
    fuelInCollectorSim -= 1;
  }
}
