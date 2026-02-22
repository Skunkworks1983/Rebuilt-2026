package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private final PIDController yawController;
  private final TargetType targetType;
  private final String logKey;

  public DriveAimedShoot(
      Drive drive,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      TargetType targetType) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetType = targetType;

    yawController =
        new PIDController(
            DriveControlConstants.YAW_CONTROL_KP, 0.0, DriveControlConstants.YAW_CONTROL_KD);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    this.logKey = "Commands/Auto Shoot/Drive Aimed " + targetType.name();

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.reset();
    Logger.recordOutput(logKey, true);
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
        new ShooterSetpoints(controls.shooterVelocity(), controls.shooterPitch()));

    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();

    Logger.recordOutput(logKey + "/Target Yaw", targetYawRadians);
    Logger.recordOutput(logKey + "/Current Yaw", currentYawRadians);

    double omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, xSupplier, ySupplier, omegaCalculated // Override rotation
            );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));
  }

  @Override
  public void end(boolean interrupted) {
    yawController.close();
    Logger.recordOutput(logKey, false);
  }
}
