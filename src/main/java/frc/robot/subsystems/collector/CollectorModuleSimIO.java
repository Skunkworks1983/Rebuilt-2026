package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.collector.collectorConstants.CollectorConstants.side;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CollectorModuleSimIO implements CollectorModuleIO {
  private IntakeSimulation collectorSimulation;

  private DCMotorSim intakeMotorUpperSim;
  private DCMotorSim intakeMotorLowerSim;
  private DCMotorSim turnMotorSim;

  private boolean isCollectorSimDown;

  final double minimumCollectorRadiansToCollect = 0.1;
  final double minimumCollectorRadiansToBeDown = 1.55;
  final double minimumCollectorSpeedToCollect = 5;

  int c = 0;

  double turnKp = 0.5;
  double turnKd = 0.4;
  double turnKi = 0.1;

  double intakeKp = 0.5;
  double intakeKd = 0;
  double intakeKi = 0;

  PIDController turnMotorPid = new PIDController(turnKp, turnKi, turnKd);
  PIDController intakeMotorsPid = new PIDController(intakeKp, intakeKi, intakeKd);

  public CollectorModuleSimIO(AbstractDriveTrainSimulation driveTrain, int collectorSide) {
    c = collectorSide;
    // collectorSide, 0 = left, 1 = right
    if (collectorSide == 0) {

      this.collectorSimulation =
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              driveTrain,
              Meters.of(0.7),
              Meters.of(0.2),
              IntakeSimulation.IntakeSide.LEFT,
              1);

    } else if (collectorSide == 1) {

      this.collectorSimulation =
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              driveTrain,
              Meters.of(0.7),
              Meters.of(0.2),
              IntakeSimulation.IntakeSide.RIGHT,
              1);
    }

    LinearSystem linearSystemIntakeMotors = LinearSystemId.createDCMotorSystem(0.002, 0.02);
    LinearSystem linearSystemTurnMotor = LinearSystemId.createDCMotorSystem(1, 1);
    double[] stdevs = {0.001, 0.001};

    intakeMotorLowerSim = new DCMotorSim(linearSystemIntakeMotors, DCMotor.getKrakenX60(1), stdevs);
    intakeMotorUpperSim = new DCMotorSim(linearSystemIntakeMotors, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim = new DCMotorSim(linearSystemTurnMotor, DCMotor.getKrakenX60(1), stdevs);
    turnMotorSim.setAngle(1.5708);
    turnMotorPid.setSetpoint(90);
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
    // Update simulations and input voltage for motors
    turnMotorSim.update(0.02);
    turnMotorSim.setInputVoltage(
        turnMotorPid.calculate(turnMotorSim.getAngularPosition().in(Degrees)));
    intakeMotorLowerSim.update(0.02);
    intakeMotorLowerSim.setInputVoltage(
        intakeMotorsPid.calculate(intakeMotorLowerSim.getAngularVelocityRadPerSec()));
    intakeMotorUpperSim.update(0.02);
    intakeMotorUpperSim.setInputVoltage(
        intakeMotorsPid.calculate(intakeMotorUpperSim.getAngularVelocityRadPerSec()));

    // Turn Motor Inputs
    input.turnConnected = true;
    input.turnPosition = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad());
    input.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
    input.turnAppliedVolts = turnMotorSim.getInputVoltage();
    input.turnCurrentAmps = turnMotorSim.getCurrentDrawAmps();

    // Lower Intake Motor Inputs
    input.lowerConnected = true;
    input.lowerPositionRad = intakeMotorLowerSim.getAngularPositionRad();
    input.lowerVelocityRadPerSec = intakeMotorLowerSim.getAngularVelocityRadPerSec();
    input.lowerAppliedVolts = intakeMotorLowerSim.getInputVoltage();
    input.lowerCurrentAmps = intakeMotorLowerSim.getCurrentDrawAmps();

    // Upper Intake Motor Inputs
    input.upperConnected = true;
    input.upperPositionRad = intakeMotorUpperSim.getAngularPositionRad();
    input.upperVelocityRadPerSec = intakeMotorUpperSim.getAngularVelocityRadPerSec();
    input.upperAppliedVolts = intakeMotorUpperSim.getInputVoltage();
    input.upperCurrentAmps = intakeMotorUpperSim.getCurrentDrawAmps();
  }

  /**
   * Gets the position of the collector
   *
   * @return
   */
  @Override
  public Angle getPosition() {
    return turnMotorSim.getAngularPosition();
  }

  /**
   * Gets the velocity of the collectors intake upper motor
   *
   * @return
   */
  @Override
  public AngularVelocity getVelocityOfIntakeLower() {
    return intakeMotorLowerSim.getAngularVelocity();
  }

  /**
   * Gets the velocity of the collectors intake lower motor
   *
   * @return
   */
  @Override
  public AngularVelocity getVelocityOfIntakeUpper() {
    return intakeMotorUpperSim.getAngularVelocity();
  }

  /**
   * Gets the velocity of the turn motor
   *
   * @return
   */
  @Override
  public AngularVelocity getVelocityOfTurn() {
    return turnMotorSim.getAngularVelocity();
  }

  /**
   * Gets the turn motors gear ratio
   *
   * @return
   */
  @Override
  public double getTurnGearRatio() {
    return turnMotorSim.getGearing();
  }

  /**
   * Sets the velocity of our intake collector motors
   *
   * @param
   */
  @Override
  public void setCollectorVelocity(AngularVelocity velocity) {

    intakeMotorsPid.setSetpoint(velocity.in(RadiansPerSecond));

    /* checks if the collector is in the right position and if the motors are spinning fast
     * enough, if both are true starts the intake of balls and if not stops it. */
    if (turnMotorSim.getAngularPositionRad() < minimumCollectorRadiansToCollect
        && intakeMotorLowerSim.getAngularVelocityRadPerSec() > minimumCollectorSpeedToCollect) {

      collectorSimulation.startIntake();
    } else {
      collectorSimulation.stopIntake();
    }
  }

  /**
   * Sets the collectors position
   *
   * @param
   */
  @Override
  public void setCollectorPos(Angle angle) {
    double angleInDegrees = angle.in(Degrees);
    turnMotorPid.setSetpoint(angleInDegrees);
  }

  /**
   * Sets the collectors turn motor velocity
   *
   * @param
   */
  @Override
  public void setCollectorTurnMotorVelocity(AngularVelocity velocity) {
    turnMotorSim.setAngularVelocity(velocity.in(RadiansPerSecond));
  }

  /**
   * Checks to see if the collector is down This is to help ensure that we only have one collector
   * down at a time
   */
  @Override
  public boolean isCollectorDown() {
    return turnMotorSim.getAngularPositionRad() < minimumCollectorRadiansToBeDown;
  }

  /** */
  @Override
  public side getSide() {
    if (c == 0) {
      return side.LEFT;
    }
    if (c == 1) {
      return side.RIGHT;
    }

    return null;
  }

  // Sets collector up
  @Override
  public void goUp() {
    setCollectorPos(Degrees.of(90));
  }

  // Sets collector down
  @Override
  public void goDown() {
    setCollectorPos(Degrees.of(0));
  }
}
