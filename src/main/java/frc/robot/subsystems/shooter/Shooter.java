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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public record ShooterSetpoints(AngularVelocity flywheelVelocity, Angle shooterPitch) {}

  private final FlywheelIO flywheel;
  private final PivotIO pivot;
  private final BeamBreakIO beamBreak;

  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Flywheel/Setpoint")
  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0.);

  @AutoLogOutput(key = "Shooter/Pivot/Setpoint")
  private Angle pivotSetpoint = Radians.of(0.);

  private final SysIdRoutine sysId;

  private static final LoggedTunableNumber flywheelKP =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kP", ShooterConstants.flywheelKP);
  private static final LoggedTunableNumber flywheelKD =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kD", ShooterConstants.flywheelKD);
  private static final LoggedTunableNumber flywheelKS =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kS", ShooterConstants.flywheelKS);
  private static final LoggedTunableNumber flywheelKV =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kV", ShooterConstants.flywheelKV);
  private static final LoggedTunableNumber flywheelKA =
      new LoggedTunableNumber("Shooter/Upper Flywheel/kA", ShooterConstants.flywheelKA);

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
   * @param flywheel The IO implementation for the flywheel.
   * @param pivot The IO implementation for the pivot.
   * @param beamBreak The IO implementation for the beam break sensor.
   */
  public Shooter(FlywheelIO flywheel, PivotIO pivot, BeamBreakIO beamBreak) {
    this.flywheel = flywheel;
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
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(flywheelInputs);
    pivot.updateInputs(pivotInputs);
    beamBreak.updateInputs(beamBreakInputs);

    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Pivot", pivotInputs);
    Logger.processInputs("Shooter/BeamBreak", beamBreakInputs);

    upperFlywheelDisconnectedAlert.set(!flywheelInputs.primaryConnected);

    // pivotDisconnectedAlert.set(
    //     (!pivotInputs.motorConnected) && Constants.currentMode != Constants.Mode.SIM);
    pivotDisconnectedAlert.set((!pivotInputs.motorConnected) && RobotBase.isSimulation() == false);
    if (Constants.tuningMode) {
      int hash = hashCode();
      if (flywheelKP.hasChanged(hash)
          || flywheelKD.hasChanged(hash)
          || flywheelKS.hasChanged(hash)
          || flywheelKV.hasChanged(hash)
          || flywheelKA.hasChanged(hash)) {
        Slot0Configs gains = new Slot0Configs();
        gains.kP = flywheelKP.get();
        gains.kD = flywheelKD.get();
        gains.kS = flywheelKS.get();
        gains.kV = flywheelKV.get();
        gains.kA = flywheelKA.get();
        flywheel.setPID(gains);

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Shooter PID Update",
                "Upper flywheel PID gains have been updated."));
      }

      if (pivotKP.hasChanged(hash)
          || pivotKD.hasChanged(hash)
          || pivotKS.hasChanged(hash)
          || pivotKV.hasChanged(hash)
          || pivotKA.hasChanged(hash)) {
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

      if (pivotMaxAcceleration.hasChanged(hash) || pivotMaxVelocity.hasChanged(hash)) {
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

    SmartDashboard.putBoolean("Shooter/Pivot/Trenchable", isPivotTrenchable());
  }

  /**
   * Gets the linear velocity of the flywheel at its effective radius.
   *
   * @return The linear velocity of the flywheel
   */
  public LinearVelocity getShooterForwardLinearVelocity() {
    return MetersPerSecond.of(
        flywheelInputs.primaryAngularVelocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters));
  }

  /**
   * Sets the flywheel to an open loop torque current
   *
   * @param output The output torque current to apply.
   */
  private void runFlywheelTorqueCurrent(double output) {
    flywheel.setOpenLoopTorqueCurrent(Amps.of(output));
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
    flywheel.setVelocity(velocity);
  }

  /**
   * Checks if the shooter is ready to shoot (flywheel and pivot at setpoint).
   *
   * @return True if the shooter is ready.
   */
  @AutoLogOutput(key = "Shooter/canShoot")
  public boolean canShoot() {
    return isFlywheelAtSetpoint() && isPivotAtSetpoint();
  }

  @AutoLogOutput(key = "Shooter/UpperFlywheelAtSetpoint")
  public boolean isFlywheelAtSetpoint() {
    if (flywheelSetpoint.lte(RotationsPerSecond.of(70))) {
      return MathUtil.isNear(
          flywheelSetpoint.in(RotationsPerSecond),
          flywheelInputs.primaryAngularVelocity.in(RotationsPerSecond),
          ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond));
    } else {
      return true;
    }
  }

  @AutoLogOutput(key = "Shooter/PivotAtSetpoint")
  public boolean isPivotAtSetpoint() {
    return MathUtil.isNear(
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
    return flywheelInputs.primaryAngularVelocity;
  }

  /**
   * Sets the shooter setpoints based on the calculated controls.
   *
   * @param setpoints The calculated shooter controls containing pitch and velocity.
   */
  public void setShooterSetpoints(ShooterSetpoints setpoints) {
    AngularVelocity targetFlywheelSetpoint = setpoints.flywheelVelocity();
    Angle targetPivotSetpoint = setpoints.shooterPitch();

    Logger.recordOutput("Shooter/Setpoints", setpoints);

    flywheelSetpoint = targetFlywheelSetpoint;
    pivotSetpoint = targetPivotSetpoint;

    pivot.setPosition(targetPivotSetpoint);
    flywheel.setVelocity(targetFlywheelSetpoint);
  }

  public void stopFlywheels() {
    flywheel.setOpenLoopTorqueCurrent(Amps.of(0.0));
  }

  public AngularVelocity getFlywheelSetpoint() {
    return flywheelSetpoint;
  }

  public boolean getBeamBreak() {
    return beamBreakInputs.beamBroken;
  }

  public void setManualAngle(Angle angle) {
    pivot.setPosition(angle);
  }

  /** Returns pivot to its default position (all the way back). */
  public void setPivotHome() {
    setManualAngle(ShooterConstants.pivotStartAngle);
  }

  public void stop() {
    stopFlywheels();
    setPivotHome();
  }

  public boolean isPivotTrenchable() {
    return pivotInputs.pivotPosition.gte(Degrees.of(ShooterConstants.pivotTrenchAngle.in(Degrees)));
  }
}
