package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
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
  private final Vision vision;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private ProfiledPIDController yawController;
  // private TrapezoidProfile profile;
  private final TargetType targetType;
  private final String logKey;
  private final String autoShootRequirementsLogKey;

  private boolean controlsLocked = false;

  public DriveAimedShoot(
      Drive drive,
      Shooter shooter,
      Vision vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      TargetType targetType) {
    this.drive = drive;
    this.shooter = shooter;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetType = targetType;

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

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.setP(DriveControlConstants.YAW_CONTROL_KP.get());
    yawController.setD(DriveControlConstants.YAW_CONTROL_KD.get());
    yawController.setConstraints(
        new Constraints(
            DriveControlConstants.YAW_MAX_VELOCITY.get(),
            DriveControlConstants.YAW_MAX_ACCELERATION.get()));
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS.get());
    yawController.reset(drive.getRotation().getRadians());

    vision.setBaselineStdDevs(
        VisionConstants.shootingLinearStdDevBaseline,
        VisionConstants.shootingAngularStdDevBaseline);

    Logger.recordOutput(logKey, true);
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);

    controlsLocked = false;
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    if (!controlsLocked) {
      ShooterControls controls =
          switch (targetType) {
            case GOAL -> ShooterCalculations.getApexConstrainedShooterControls(
                currentPose, currentSpeeds);
            case TRANSFER -> ShooterCalculations.getShooterControlsTransfer(
                currentSpeeds, currentPose, shooter.getShooterForwardLinearVelocity());
          };

      shooter.setShooterSetpoints(
          new ShooterSetpoints(controls.flywheelVelocity(), controls.shooterPitch()));
      double currentYawRadians = drive.getRotation().getRadians();
      double targetYawRadians = controls.driveYaw().getRadians();

      Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
      Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);

      double omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);

      Logger.recordOutput("Auto Shoot/Omega Calculated", omegaCalculated);

      ChassisSpeeds targetSpeeds =
          DriveUtils.getChassisSpeeds(
              drive, xSupplier, ySupplier, omegaCalculated // Override rotation
              );

      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

      Rotation2d fieldCentricOffset =
          isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));
    }

    Logger.recordOutput(
        autoShootRequirementsLogKey + "/DriveReadyToShoot", yawController.atSetpoint());
    Logger.recordOutput(
        autoShootRequirementsLogKey + "/ReadyToShoot",
        yawController.atSetpoint() && shooter.canShoot());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivotHome();
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/TriggerCommandScheduled", false);
    Logger.recordOutput(logKey, false);

    vision.setBaselineStdDevs(
        VisionConstants.teleopLinearStdDevBaseline, VisionConstants.teleopAngularStdDevBaseline);
  }

  public void lockControls() {
    controlsLocked = true;
  }

  public void unlockControls() {
    controlsLocked = false;
  }
}
