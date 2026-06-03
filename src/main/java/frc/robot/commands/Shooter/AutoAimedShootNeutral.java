package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveControlConstants;
import frc.robot.commands.DriveUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class AutoAimedShootNeutral extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Command feedCommand;

  private final ProfiledPIDController yawController;

  private final Debouncer indexDebounce = new Debouncer(0.5);

  private final ShooterControls controls =
      new ShooterControls(
          ShooterConstants.defaultVelocityTransferClose,
          ShooterConstants.defaultVelocityTransferClose,
          ShooterConstants.defaultPitchTransferClose,
          new Rotation2d(Degrees.of(180)),
          true);

  private boolean feedEnabled = false;

  public AutoAimedShootNeutral(Drive drive, Shooter shooter, Command feedCommand) {
    this.drive = drive;
    this.shooter = shooter;
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

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    yawController.reset(drive.getRotation().getRadians());
    feedEnabled = false;
    Logger.recordOutput("Auto Shoot/Feed Enabled", false);
    Logger.recordOutput("Commands/Auto Aimed Shoot Neutral", true);

    shooter.setShooterSetpoints(
        new ShooterSetpoints(controls.flywheelVelocity(), controls.shooterPitch()));
  }

  @Override
  public void execute() {
    if (!feedEnabled) {
      double currentYawRadians = drive.getRotation().getRadians();
      double targetYawRadians = controls.driveYaw().getRadians();

      Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
      Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);
      double omegaCalculated;

      omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);
      Logger.recordOutput("Auto Shoot/Omega Calculated", omegaCalculated);

      ChassisSpeeds targetSpeeds =
          DriveUtils.getChassisSpeeds(
              drive, () -> 0, () -> 0, omegaCalculated // Override rotation
              );

      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

      Rotation2d fieldCentricOffset =
          isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

      Logger.recordOutput("Auto Shoot/At Setpoint", yawController.atSetpoint());
      Logger.recordOutput("Auto Shoot/At Goal", yawController.atGoal());
      Logger.recordOutput(
          "Auto Shoot/Debounced", indexDebounce.calculate(yawController.atSetpoint()));

      boolean readyToFeed = yawController.atGoal();
      Logger.recordOutput("Auto Shoot/Trigger Ready", readyToFeed);
      if (readyToFeed) {
        feedEnabled = true;
        Logger.recordOutput("Auto Shoot/Feed Enabled", true);
        drive.stopWithX();
        CommandScheduler.getInstance().schedule(feedCommand);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    Logger.recordOutput("Commands/Auto Aimed Shoot Neutral", false);
    feedCommand.cancel();

    feedEnabled = false;
  }
}
