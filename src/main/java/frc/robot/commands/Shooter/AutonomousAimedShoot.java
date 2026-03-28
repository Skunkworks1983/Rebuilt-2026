package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveControlConstants;
import frc.robot.commands.DriveUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.IndexerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class AutonomousAimedShoot extends Command {
  private enum AutoUnjamState {
    FEED,
    REVERSE
  }

  private final Drive drive;
  private final Shooter shooter;
  private final Hopper hopper;

  private final ProfiledPIDController yawController;

  private final Debouncer indexDebounce = new Debouncer(0.5);
  private final Timer autoUnjamTimer = new Timer();

  private AutoUnjamState autoUnjamState = AutoUnjamState.FEED;
  private boolean feedEnabled = false;

  public AutonomousAimedShoot(Drive drive, Shooter shooter, Hopper hopper) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    yawController =
        new ProfiledPIDController(
            DriveControlConstants.YAW_CONTROL_KP.get(),
            0.0,
            DriveControlConstants.YAW_CONTROL_KD.get(),
            new Constraints(
                DriveControlConstants.YAW_MAX_VELOCITY.get(),
                DriveControlConstants.YAW_MAX_ACCELERATION.get()));
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS.get());

    addRequirements(shooter, hopper);
  }

  @Override
  public void initialize() {
    yawController.reset(drive.getRotation().getRadians());
    autoUnjamTimer.stop();
    autoUnjamTimer.reset();
    autoUnjamState = AutoUnjamState.FEED;
    feedEnabled = false;
    hopper.setIndexerDutyCycle(0.0);
    Logger.recordOutput("Auto Shoot/Feed Enabled", false);
    Logger.recordOutput("Auto Shoot/Unjamming", false);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    ShooterControls controls =
        ShooterCalculations.getApexConstrainedShooterControls(currentPose, currentSpeeds);

    shooter.setShooterSetpoints(
        new ShooterSetpoints(
            controls.upperFlywheelVelocity(),
            controls.lowerFlywheelVelocity(),
            controls.shooterPitch()));
    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();
    // double targetYawDelta = MathUtil.angleModulus(targetYawRadians - lastFilteredYaw);

    // TrapezoidProfile.State setpoint =
    //     profile.calculate(
    //         0.02,
    //         new TrapezoidProfile.State(
    //             currentYawRadians, drive.getAngularVelocity().in(RadiansPerSecond)),
    //         new TrapezoidProfile.State(targetYawRadians, 0));

    // if (Math.abs(targetYawDelta) > DriveControlConstants.largeAngleJumpThreshold.in(Radians)) {
    //   lastFilteredYaw = targetYawRadians;
    // } else {
    //   lastFilteredYaw += yawFilterAlpha * targetYawDelta;
    // }
    // double filteredTargetYaw = MathUtil.angleModulus(lastFilteredYaw);
    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);
    // Logger.recordOutput("Auto Shoot/Filtered Target Yaw", filteredTargetYaw);
    double omegaCalculated;

    omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);
    Logger.recordOutput("Auto Shoot/Omega Calculated", omegaCalculated);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, () -> 0, () -> 0, omegaCalculated // Override rotation
            );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

    Logger.recordOutput("Auto Shoot/At Setpoint", yawController.atSetpoint());
    Logger.recordOutput("Auto Shoot/At Goal", yawController.atGoal());
    Logger.recordOutput(
        "Auto Shoot/Debounced", indexDebounce.calculate(yawController.atSetpoint()));

    boolean readyToFeed = yawController.atGoal() && shooter.isUpperFlywheelAtSetpoint();
    Logger.recordOutput("Auto Shoot/Indexer Ready", readyToFeed);

    if (readyToFeed && !feedEnabled) {
      feedEnabled = true;
      autoUnjamState = AutoUnjamState.FEED;
      autoUnjamTimer.restart();
      Logger.recordOutput("Auto Shoot/Feed Enabled", true);
      applyAutoUnjamOutput();
    }

    if (feedEnabled) {
      handleAutoUnjam(shooter.getBeamBreak());
    } else {
      hopper.setIndexerDutyCycle(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    hopper.setIndexerDutyCycle(0);
    autoUnjamTimer.stop();
    feedEnabled = false;
    Logger.recordOutput("Auto Shoot/Feed Enabled", false);
    Logger.recordOutput("Auto Shoot/Unjamming", false);
  }

  private void handleAutoUnjam(boolean beamBroken) {
    switch (autoUnjamState) {
      case FEED:
        if (!beamBroken) {
          autoUnjamTimer.restart();
        } else if (autoUnjamTimer.hasElapsed(
            IndexerConstants.autoUnjamNoBeamBreakTimeoutSeconds.get())) {
          setAutoUnjamState(AutoUnjamState.REVERSE);
        }
        break;
      case REVERSE:
        if (autoUnjamTimer.hasElapsed(IndexerConstants.autoUnjamReverseDurationSeconds.get())) {
          setAutoUnjamState(AutoUnjamState.FEED);
        }
        break;
    }
  }

  private void setAutoUnjamState(AutoUnjamState nextState) {
    if (autoUnjamState == nextState) {
      return;
    }

    autoUnjamState = nextState;
    autoUnjamTimer.restart();
    applyAutoUnjamOutput();
  }

  private void applyAutoUnjamOutput() {

    double speedRps = IndexerConstants.indexerAngularVelocityRotationsPerSecond.get();

    if (autoUnjamState == AutoUnjamState.FEED) {
      hopper.setIndexerVelocity(RotationsPerSecond.of(speedRps));
      Logger.recordOutput("Auto Shoot/Unjamming", false);
      Logger.recordOutput("Auto Shoot/UnjamState", "FEED");
      Logger.recordOutput("Auto Shoot/Indexer Velocity", speedRps);
    } else {
      hopper.setIndexerVelocity(RotationsPerSecond.of(-speedRps));
      Logger.recordOutput("Auto Shoot/Unjamming", true);
      Logger.recordOutput("Auto Shoot/UnjamState", "REVERSE");
      Logger.recordOutput("Auto Shoot/Indexer Velocity", -50);
    }
  }
}
