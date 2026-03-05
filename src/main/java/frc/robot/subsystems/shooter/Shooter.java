package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem responsible for the shooter mechanism, including the flywheel and pivot. Handles
 * pivoting, spinning up the flywheel, and sysid characterization the mechanism.
 *
 * @author Eddy Walsh
 */
public class Shooter extends SubsystemBase {
  /**
   * Data record representing the calculated control outputs for the shooter subsystem.
   *
   * @param shooterVelocity Target flywheel angular velocity.
   * @param shooterPitch Target pivot angle.
   */
  public record ShooterSetpoints(
      AngularVelocity upperFlywheelVelocity,
      AngularVelocity lowerFlywheelVelocity,
      Angle shooterPitch) {}

  private final FlywheelIO upperFlywheel;
  private final FlywheelIO lowerFlywheel;

  private final PivotIO pivot;

  private final BeamBreakIO beamBreak;

  private final FlywheelIOInputsAutoLogged upperFlywheelInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged lowerFlywheelInputs = new FlywheelIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0.);
  private Angle pivotSetpoint = Radians.of(0.);

  private final SysIdRoutine sysId;

  private static final LoggedTunableNumber upperFlywheelKP =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kP", ShooterConstants.upperFlywheelKP);
  private static final LoggedTunableNumber upperFlywheelKD =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kD", ShooterConstants.upperFlywheelKD);
  private static final LoggedTunableNumber upperFlywheelKS =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kS", ShooterConstants.upperFlywheelKS);
  private static final LoggedTunableNumber upperFlywheelKV =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kV", ShooterConstants.upperFlywheelKV);
  private static final LoggedTunableNumber upperFlywheelKA =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kA", ShooterConstants.upperFlywheelKA);

  private static final LoggedTunableNumber lowerFlywheelKP =
      new LoggedTunableNumber("Shooter/Lower Flywheel/kP", ShooterConstants.lowerFlywheelKP);
  private static final LoggedTunableNumber lowerFlywheelKD =
      new LoggedTunableNumber("Shooter/Lower Flywheel/kD", ShooterConstants.lowerFlywheelKD);
  private static final LoggedTunableNumber lowerFlywheelKS =
      new LoggedTunableNumber("Shooter/Lower Flywheel/kS", ShooterConstants.lowerFlywheelKS);
  private static final LoggedTunableNumber lowerFlywheelKV =
      new LoggedTunableNumber("Shooter/Lower Flywheel/kV", ShooterConstants.lowerFlywheelKV);
  private static final LoggedTunableNumber lowerFlywheelKA =
      new LoggedTunableNumber("Shooter/Lower Flywheel/kA", ShooterConstants.lowerFlywheelKA);

  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("Shooter/Pivot/kP", ShooterConstants.pivotkP);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("Shooter/Pivot/kD", ShooterConstants.pivotkD);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("Shooter/Pivot/kS", ShooterConstants.pivotkS);
  private static final LoggedTunableNumber pivotKV =
      new LoggedTunableNumber("Shooter/Pivot/kV", ShooterConstants.pivotkV);
  private static final LoggedTunableNumber pivotKA =
      new LoggedTunableNumber("Shooter/Pivot/kA", ShooterConstants.pivotkA);

  private static final LoggedTunableNumber pivotMaxVelocity =
      new LoggedTunableNumber(
          "Shooter/Pivot/Max Velocity [rps]",
          ShooterConstants.pivotMaxVelocity.in(RotationsPerSecond));
  private static final LoggedTunableNumber pivotMaxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Pivot/Max Acceleration [rps^2]",
          ShooterConstants.pivotMaxAcceleration.in(RotationsPerSecondPerSecond));

  private final Alert upperFlywheelDisconnectedAlert =
      new Alert("Forward Flywheel disconnected", AlertType.kError);
  private final Alert lowerFlywheelDisconnectedAlert =
      new Alert("Back Flywheel disconnected", AlertType.kError);
  private final Alert pivotDisconnectedAlert = new Alert("Pivot disconnected", AlertType.kError);

  /**
   * Constructs a new Shooter instance.
   *
   * @param forwardFlywheel The IO implementation for the forward (launch) flywheel.
   * @param forwardFlywheel The IO implementation for the back flywheel.
   * @param pivot The IO implementation for the pivot.
   * @param beamBreak The IO implementation for the beam break sensor.
   */
  public Shooter(
      FlywheelIO forwardFlywheel, FlywheelIO backFlywheel, PivotIO pivot, BeamBreakIO beamBreak) {
    this.upperFlywheel = forwardFlywheel;
    this.lowerFlywheel = backFlywheel;
    this.pivot = pivot;
    this.beamBreak = beamBreak;

    // Allows for system identification of the flywheel via auto routines (finds
    // torque current
    // required to overcome static friction, current for one unit of velocity, and
    // current for one
    // unit of acceleration)
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                ShooterConstants.sysIdRampRate,
                ShooterConstants.sysIdStepSize,
                null,
                (state) -> SignalLogger.writeString("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runFlywheelTorqueCurrent(voltage.in(Volts)), null, this));

    // This ensures upper flywheel gets upper PID and lower flywheel gets lower PID
    // (the io defaults to upper)
    upperFlywheel.setPID(
        new Slot0Configs()
            .withKP(ShooterConstants.upperFlywheelKP)
            .withKD(ShooterConstants.upperFlywheelKD)
            .withKS(ShooterConstants.upperFlywheelKS)
            .withKV(ShooterConstants.upperFlywheelKV)
            .withKA(ShooterConstants.upperFlywheelKA));
    lowerFlywheel.setPID(
        new Slot0Configs()
            .withKP(ShooterConstants.lowerFlywheelKP)
            .withKD(ShooterConstants.lowerFlywheelKD)
            .withKS(ShooterConstants.lowerFlywheelKS)
            .withKV(ShooterConstants.lowerFlywheelKV)
            .withKA(ShooterConstants.lowerFlywheelKA));
  }

  @Override
  public void periodic() {
    upperFlywheel.updateInputs(upperFlywheelInputs);
    lowerFlywheel.updateInputs(lowerFlywheelInputs);
    pivot.updateInputs(pivotInputs);
    beamBreak.updateInputs(beamBreakInputs);

    Logger.processInputs("Shooter/Flywheel/Lower Flywheel", lowerFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Upper Flywheel", upperFlywheelInputs);
    Logger.processInputs("Shooter/Pivot", pivotInputs);
    Logger.processInputs("Shooter/BeamBreak", beamBreakInputs);

    upperFlywheelDisconnectedAlert.set(!upperFlywheelInputs.connected);

    lowerFlywheelDisconnectedAlert.set(!lowerFlywheelInputs.connected);

    pivotDisconnectedAlert.set(
        (!pivotInputs.motorConnected) && Constants.currentMode != Constants.Mode.SIM);

    if (upperFlywheelKP.hasChanged(hashCode())
        || upperFlywheelKD.hasChanged(hashCode())
        || upperFlywheelKS.hasChanged(hashCode())
        || upperFlywheelKV.hasChanged(hashCode())
        || upperFlywheelKA.hasChanged(hashCode())) {
      Slot0Configs gains = new Slot0Configs();
      gains.kP = upperFlywheelKP.get();
      gains.kD = upperFlywheelKD.get();
      gains.kS = upperFlywheelKS.get();
      gains.kV = upperFlywheelKV.get();
      gains.kA = upperFlywheelKA.get();
      upperFlywheel.setPID(gains);

      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Shooter PID Update",
              "Upper flywheel PID gains have been updated."));
    }

    if (lowerFlywheelKP.hasChanged(hashCode())
        || lowerFlywheelKD.hasChanged(hashCode())
        || lowerFlywheelKS.hasChanged(hashCode())
        || lowerFlywheelKV.hasChanged(hashCode())
        || lowerFlywheelKA.hasChanged(hashCode())) {
      Slot0Configs gains = new Slot0Configs();
      gains.kP = lowerFlywheelKP.get();
      gains.kD = lowerFlywheelKD.get();
      gains.kS = lowerFlywheelKS.get();
      gains.kV = lowerFlywheelKV.get();
      gains.kA = lowerFlywheelKA.get();
      lowerFlywheel.setPID(gains);

      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Shooter PID Update",
              "Lower flywheel PID gains have been updated."));
    }

    if (pivotKP.hasChanged(hashCode())
        || pivotKD.hasChanged(hashCode())
        || pivotKS.hasChanged(hashCode())
        || pivotKV.hasChanged(hashCode())
        || pivotKA.hasChanged(hashCode())) {
      Slot0Configs gains = new Slot0Configs();
      gains.kP = pivotKP.get();
      gains.kD = pivotKD.get();
      gains.kS = pivotKS.get();
      gains.kV = pivotKV.get();
      gains.kA = pivotKA.get();
      pivot.setPID(gains);

      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Shooter PID Update",
              "Pivot PID gains have been updated."));
    }

    if (pivotMaxAcceleration.hasChanged(hashCode()) || pivotMaxVelocity.hasChanged(hashCode())) {
      pivot.setMotionConstraints(
          RotationsPerSecond.of(pivotMaxVelocity.get()),
          RotationsPerSecondPerSecond.of(pivotMaxAcceleration.get()));

      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Shooter Motion Constraints Update",
              "Pivot motion constraints have been updated."));
    }
  }

  /**
   * Gets the linear velocity of the flywheel at its effective radius.
   *
   * @return The linear velocity of the flywheel
   */
  public LinearVelocity getShooterForwardLinearVelocity() {
    return MetersPerSecond.of(
        upperFlywheelInputs.angularVelocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters));
  }

  /**
   * Sets the flywheel to an open loop torque current
   *
   * @param output The output torque current to apply.
   */
  private void runFlywheelTorqueCurrent(double output) {
    upperFlywheel.setOpenLoopTorqueCurrent(Amps.of(output));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runFlywheelTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runFlywheelTorqueCurrent(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  /**
   * Sets the flywheel angular velocity manually.
   *
   * @param velocity The angular velocity to set.
   */
  public void setManualVelocity(AngularVelocity velocity) {
    upperFlywheel.setVelocity(velocity);
    lowerFlywheel.setVelocity(velocity);
  }

  /**
   * Checks if the shooter is ready to shoot (flywheel and pivot at setpoint).
   *
   * @return True if the shooter is ready.
   */
  @AutoLogOutput(key = "Shooter/canShoot")
  public boolean canShoot() {
    return MathUtil.isNear(
            flywheelSetpoint.in(RotationsPerSecond),
            upperFlywheelInputs.angularVelocity.in(RotationsPerSecond),
            ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond))
        && MathUtil.isNear(
            flywheelSetpoint.in(RotationsPerSecond),
            lowerFlywheelInputs.angularVelocity.in(RotationsPerSecond),
            ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond))
        && MathUtil.isNear(
            pivotSetpoint.in(Degrees),
            pivotInputs.pivotPosition.in(Degrees),
            ShooterConstants.pivotPositionTolerence.in(Degrees));
  }

  /**
   * Gets the current pivot angle.
   *
   * @return The pivot Angle.
   */
  public Angle getPivotAngle() {
    return pivotInputs.pivotPosition;
  }

  /**
   * Gets the current flywheel velocity.
   *
   * @return The flywheel AngularVelocity.
   */
  public AngularVelocity getFlywheelVelocity() {
    return upperFlywheelInputs.angularVelocity;
  }

  /**
   * Sets the shooter setpoints based on the calculated controls.
   *
   * @param setpoints The calculated shooter controls containing pitch and velocity.
   */
  public void setShooterSetpoints(ShooterSetpoints setpoints) {
    AngularVelocity upperFlywheelSetpoint = setpoints.upperFlywheelVelocity();
    AngularVelocity lowerFlywheelSetpoint = setpoints.lowerFlywheelVelocity();
    Angle pivotSetpoint = setpoints.shooterPitch();

    Logger.recordOutput("Shooter/Setpoints", setpoints);

    pivot.setPosition(pivotSetpoint);
    upperFlywheel.setVelocity(upperFlywheelSetpoint);
    lowerFlywheel.setVelocity(lowerFlywheelSetpoint);
  }

  public void stopFlywheels() {
    upperFlywheel.setOpenLoopTorqueCurrent(Amps.of(0.0));
    lowerFlywheel.setOpenLoopTorqueCurrent(Amps.of(0.0));
  }

  public boolean getBeamBreak() {
    return beamBreakInputs.beamBroken;
  }

  public void setManualAngle(Angle angle) {
    pivot.setPosition(angle);
  }
}
