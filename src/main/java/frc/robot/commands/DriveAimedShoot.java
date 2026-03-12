package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedShoot extends Command {
  public enum TargetType {
    GOAL,
    TRANSFER
  }

  private final Drive drive;
  private final Shooter shooter;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotationSupplier;
  private ProfiledPIDController yawController;
  // private TrapezoidProfile profile;
  private final TargetType targetType;
  private final String logKey;
  private final String autoShootRequirementsLogKey;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(2, 2.5);

  private double lastFilteredYaw;
  private double yawFilterAlpha;

  public DriveAimedShoot(
      Drive drive,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      TargetType targetType) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.targetType = targetType;

    // profile =
    // new TrapezoidProfile(
    // new TrapezoidProfile.Constraints(
    // DriveControlConstants.YAW_MAX_VELOCITY.get(),
    // DriveControlConstants.YAW_MAX_ACCELERATION.get()));

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

    this.logKey = "Commands/Auto Shoot/Drive Aimed " + targetType.name();
    this.autoShootRequirementsLogKey = logKey + "/Requirements";
    // readyToShoot =
    //     new Trigger(() -> shooter.canShoot() && yawController.atSetpoint() && commandStarted);
    // readyToShoot.whileTrue(indexerCommand);
    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
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

    // TODO: Test if filtering is helpful
    // lastFilteredYaw = drive.getRotation().getRadians();
    // double filterPeriod = DriveControlConstants.linearFilterPeriod.get();
    // double filterTimeConstant =
    // DriveControlConstants.linearFilterTimeConstant.get();
    // yawFilterAlpha =
    // filterTimeConstant <= 0.0
    // ? 1.0
    // : MathUtil.clamp(filterPeriod / (filterTimeConstant + filterPeriod), 0.0,
    // 1.0);
    Logger.recordOutput(logKey, true);
    // Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    yawController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    ShooterControls controls =
        switch (targetType) {
          case GOAL -> ShooterCalculations.getApexConstrainedShooterControls(
              currentPose, currentSpeeds);
          case TRANSFER -> ShooterCalculations.getShooterControlsTransfer(
              currentSpeeds, currentPose, shooter.getShooterForwardLinearVelocity());
        };

    shooter.setShooterSetpoints(
        new ShooterSetpoints(
            controls.upperFlywheelVelocity(),
            controls.lowerFlywheelVelocity(),
            controls.shooterPitch()));
    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();
    // double targetYawDelta = MathUtil.angleModulus(targetYawRadians -
    // lastFilteredYaw);

    // TrapezoidProfile.State setpoint =
    // profile.calculate(
    // 0.02,
    // new TrapezoidProfile.State(
    // currentYawRadians, drive.getAngularVelocity().in(RadiansPerSecond)),
    // new TrapezoidProfile.State(targetYawRadians, 0));

    // if (Math.abs(targetYawDelta) >
    // DriveControlConstants.largeAngleJumpThreshold.in(Radians)) {
    // lastFilteredYaw = targetYawRadians;
    // } else {
    // lastFilteredYaw += yawFilterAlpha * targetYawDelta;
    // }
    // double filteredTargetYaw = MathUtil.angleModulus(lastFilteredYaw);
    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);
    // Logger.recordOutput("Auto Shoot/Filtered Target Yaw", filteredTargetYaw);
    double omegaCalculated;

    if (targetType == TargetType.GOAL) {

      omegaCalculated =
          yawController.calculate(currentYawRadians, targetYawRadians)
              + rotationSupplier.getAsDouble();
    } else {
      omegaCalculated = rotationSupplier.getAsDouble();
    }
    Logger.recordOutput("Auto Shoot/Omega Calculated", omegaCalculated);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, xSupplier, ySupplier, omegaCalculated // Override rotation
            );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

    Logger.recordOutput(
        autoShootRequirementsLogKey + "/DriveReadyToShoot", yawController.atSetpoint());
    Logger.recordOutput(
        autoShootRequirementsLogKey + "/ReadyToShoot",
        yawController.atSetpoint() && shooter.canShoot());
  }

  @Override
  public void end(boolean interrupted) {
    // CommandScheduler.getInstance().cancel(indexerCommand);
    shooter.setPivotHome();
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(logKey, false);
  }
}
