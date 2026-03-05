package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Command indexerCommand;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController yawController;
  private final TargetType targetType;
  private final String logKey;
  private final String autoShootRequirementsLogKey;
  private boolean wasReadyToShoot = false;

  private Trigger readyToShoot;

  public DriveAimedShoot(
      Drive drive,
      Shooter shooter,
      Command indexerCommand,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      TargetType targetType) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetType = targetType;
    this.indexerCommand = indexerCommand;

    yawController = new PIDController(
        DriveControlConstants.YAW_CONTROL_KP, 0.0, DriveControlConstants.YAW_CONTROL_KD);
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS);

    this.logKey = "Commands/Auto Shoot/Drive Aimed " + targetType.name();
    this.autoShootRequirementsLogKey = logKey + "/Requirements";
    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.reset();
    wasReadyToShoot = false;
    CommandScheduler.getInstance().cancel(indexerCommand);
    Logger.recordOutput(logKey, true);
    Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    readyToShoot = new Trigger(() -> shooter.canShoot() && yawController.atSetpoint());
    readyToShoot.whileTrue(indexerCommand);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    ShooterControls controls = switch (targetType) {
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

    Logger.recordOutput(logKey + "/Target Yaw", targetYawRadians);
    Logger.recordOutput(logKey + "/Current Yaw", currentYawRadians);

    double omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);

    ChassisSpeeds targetSpeeds = DriveUtils.getChassisSpeeds(
        drive, xSupplier, ySupplier, omegaCalculated // Override rotation
    );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

    Logger.recordOutput(autoShootRequirementsLogKey + "/YawErrorRadians", yawController.getError());
    Logger.recordOutput(autoShootRequirementsLogKey + "/DriveReadyToShoot", yawController.atSetpoint());
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", yawController.atSetpoint() && shooter.canShoot());
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(indexerCommand);
    shooter.setPivotHome();
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(logKey, false);
    readyToShoot = null;
  }
}
