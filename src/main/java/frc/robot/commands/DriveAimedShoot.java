package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private PIDController yawController;
  private TrapezoidProfile profile;
  private final TargetType targetType;
  private final String logKey;
  private final String autoShootRequirementsLogKey;
  private boolean commandStarted = false;

  private double lastFilteredYaw;
  private double yawFilterAlpha;

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

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DriveControlConstants.YAW_MAX_VELOCITY.get(),
                DriveControlConstants.YAW_MAX_ACCELERATION.get()));

    yawController =
        new PIDController(
            DriveControlConstants.YAW_CONTROL_KP.get(),
            0.0,
            DriveControlConstants.YAW_CONTROL_KD.get());
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS.get());

    this.logKey = "Commands/Auto Shoot/Drive Aimed " + targetType.name();
    this.autoShootRequirementsLogKey = logKey + "/Requirements";
    readyToShoot =
        new Trigger(() -> shooter.canShoot() && yawController.atSetpoint() && commandStarted);
    readyToShoot.whileTrue(indexerCommand);
    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController =
        new PIDController(
            DriveControlConstants.YAW_CONTROL_KP.get(),
            0.0,
            DriveControlConstants.YAW_CONTROL_KD.get());
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS.get());

    CommandScheduler.getInstance().cancel(indexerCommand);

    lastFilteredYaw = drive.getRotation().getRadians();
    double filterPeriod = DriveControlConstants.linearFilterPeriod.get();
    double filterTimeConstant = DriveControlConstants.linearFilterTimeConstant.get();
    yawFilterAlpha =
        filterTimeConstant <= 0.0
            ? 1.0
            : MathUtil.clamp(filterPeriod / (filterTimeConstant + filterPeriod), 0.0, 1.0);
    commandStarted = true;
    Logger.recordOutput(logKey, true);
    Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
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

    TrapezoidProfile.State setpoint =
        profile.calculate(
            0.02,
            new TrapezoidProfile.State(
                currentYawRadians, drive.getAngularVelocity().in(RadiansPerSecond)),
            new TrapezoidProfile.State(targetYawRadians, 0));

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

    double omegaCalculated = yawController.calculate(currentYawRadians, setpoint.position);

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
    commandStarted = false;
    CommandScheduler.getInstance().cancel(indexerCommand);
    shooter.setPivotHome();
    Logger.recordOutput(autoShootRequirementsLogKey + "/ReadyToShoot", false);
    Logger.recordOutput(autoShootRequirementsLogKey + "/IndexerCommandScheduled", false);
    Logger.recordOutput(logKey, false);
  }
}
