package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedShoot extends Command {
  public enum TargetType {
    GOAL,
    TRANSFER_NEUTRAL,
    TRANSFER_FAR
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

  private final BooleanSupplier indexPermitted;
  private final Command feedCommand;

  private boolean shooterSetpointsLocked = false;
  private boolean feeding = false;

  private final LoggedTunableNumber transferFarVelocity =
      new LoggedTunableNumber(
          "Drive Aimed Shoot/Transfer Far Velocity RPS",
          ShooterConstants.defaultVelocityTransferFar.in(RotationsPerSecond));
  private final LoggedTunableNumber transferFarPitch =
      new LoggedTunableNumber(
          "Drive Aimed Shoot/Transfer Far Pitch Degrees",
          ShooterConstants.defaultPitchTransferFar.in(Degrees));
  private final LoggedTunableNumber transferCloseVelocity =
      new LoggedTunableNumber(
          "Drive Aimed Shoot/Transfer Close Velocity RPS",
          ShooterConstants.defaultVelocityTransferClose.in(RotationsPerSecond));
  private final LoggedTunableNumber transferClosePitch =
      new LoggedTunableNumber(
          "Drive Aimed Shoot/Transfer Close Pitch Degrees",
          ShooterConstants.defaultPitchTransferClose.in(Degrees));

  public DriveAimedShoot(
      Drive drive,
      Shooter shooter,
      Vision vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      TargetType targetType,
      BooleanSupplier indexPermitted,
      Command feedCommand) {
    this.drive = drive;
    this.shooter = shooter;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetType = targetType;
    this.indexPermitted = indexPermitted;
    this.feedCommand = feedCommand;

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

    shooterSetpointsLocked = false;
    feeding = false;
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();
    ShooterControls controls =
        switch (targetType) {
          case GOAL -> ShooterCalculations.getApexConstrainedShooterControls(
              currentPose, currentSpeeds);
          case TRANSFER_NEUTRAL -> new ShooterControls(
              RotationsPerSecond.of(transferCloseVelocity.get()),
              Degrees.of(transferClosePitch.get()),
              Rotation2d.fromDegrees(180),
              true);
          case TRANSFER_FAR -> new ShooterControls(
              RotationsPerSecond.of(transferFarVelocity.get()),
              Degrees.of(transferFarPitch.get()),
              Rotation2d.fromDegrees(180),
              true);
        };

    if (!shooterSetpointsLocked) {
      shooter.setShooterSetpoints(
          new ShooterSetpoints(controls.flywheelVelocity(), controls.shooterPitch()));
    }
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

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

    if (shooter.canShoot() && indexPermitted.getAsBoolean() && !feeding) {
      if (targetType == TargetType.GOAL) {
        shooterSetpointsLocked = true;
      }
      feeding = true;
      CommandScheduler.getInstance().schedule(feedCommand);

      new Trigger(indexPermitted)
          .onFalse(
              new InstantCommand(
                  () -> {
                    CommandScheduler.getInstance().cancel(feedCommand);
                    shooterSetpointsLocked = false;
                    feeding = false;
                  }));
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
    CommandScheduler.getInstance().cancel(feedCommand);

    shooterSetpointsLocked = false;
    feeding = false;

    vision.setBaselineStdDevs(
        VisionConstants.teleopLinearStdDevBaseline, VisionConstants.teleopAngularStdDevBaseline);
  }

  public void lockControls() {
    shooterSetpointsLocked = true;
    drive.stopWithX();
  }

  public void unlockControls() {
    shooterSetpointsLocked = false;
  }
}
