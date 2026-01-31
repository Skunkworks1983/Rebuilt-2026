package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CollectorModuleSimIO implements CollectorModuleIO {
  private IntakeSimulation collectorSimulation;

  private DCMotorSim intakeMotorUpperSim;
  private DCMotorSim intakeMotorLowerSim;
  private DCMotorSim turnMotorSim;

  private boolean isCollectorSimDown;

  final double minimumCollectorRotationToCollect = 1.4;
  final double minimumCollectorSpeedToCollect = 1;

  public CollectorModuleSimIO(AbstractDriveTrainSimulation driveTrain, int collectorSide) {

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

    LinearSystem linearSystem = LinearSystemId.createDCMotorSystem(1, 1);
    double[] stdevs = {0.001, 0.001};

    intakeMotorLowerSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX44(1), stdevs);
    intakeMotorUpperSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX44(1), stdevs);
    turnMotorSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX44(1), stdevs);
  }

  /**
   * detects whether fuel is inside collector
   *
   * @return
   */
  @Override
  public boolean isFuelInsideCollector() {
    return collectorSimulation.getGamePiecesAmount() != 0;
  }

  /**
   * Updates our inputs for all motors
   *
   * @param
   */
  @Override
  public void updateInputs(CollectorIOInputs input) {
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
    intakeMotorLowerSim.setAngularVelocity(velocity.in(RadiansPerSecond));
    intakeMotorUpperSim.setAngularVelocity(velocity.in(RadiansPerSecond));

    /* checks if the collector is in the right position and if the motors are spinning fast
     * enough, if both are true starts the intake of balls and if not stops it. */
    if (turnMotorSim.getAngularPositionRad() > minimumCollectorRotationToCollect
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
    turnMotorSim.setAngle(angle.in(Radians));
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
   * sets the collector isDown. This is to help ensure that we only have one collector down at a
   * time
   */
  @Override
  public void setIsCollectorDown(boolean downed) {
    isCollectorSimDown = downed;
  }

  /**
   * Checks to see if the collector is down This is to help ensure that we only have one collector
   * down at a time
   */
  @Override
  public boolean isCollectorDown() {
    if (turnMotorSim.getAngularPositionRad() > minimumCollectorRotationToCollect) {
      isCollectorSimDown = true;
    } else {
      isCollectorSimDown = false;
    }
    return isCollectorSimDown;
  }
}
