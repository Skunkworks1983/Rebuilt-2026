package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedGoal extends Command {
  /**
   * Uses odometry to continuously aim at the hub and keep flywheel/pivot in correct state to make a
   * shot into the hub. Uses the apex constrained solver ({@link
   * frc.robot.util.ShooterCalculations#getApexConstrainedShooterControls}).
   *
   * @see {@link frc.robot.util.ShooterCalculations#getApexConstrainedShooterControls}
   */
  private final Drive drive;

  private final Shooter shooter;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController driveYawController;

  public DriveAimedGoal(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    driveYawController =
        new PIDController(
            DriveControlConstants.yawControlKp, 0.0, DriveControlConstants.yawControlKD);
    driveYawController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    // Reset PID when command initializes to prevent "kick"
    driveYawController.reset();
  }

  @Override
  public void execute() {
    ShooterControls controls =
        ShooterCalculations.getApexConstrainedShooterControls(
            drive.getPose(), drive.getChassisSpeeds(), drive.getAngularVelocity());
    shooter.setShooterControls(controls);

    Logger.recordOutput("Auto Shoot/Drive Yaw Setpoint", controls.driveYaw().getRadians(), Radians);
    double omegaSetpoint =
        driveYawController.calculate(
            drive.getRotation().getRadians(), controls.driveYaw().getRadians());

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    ChassisSpeeds desiredSpeeds =
        DriveUtils.getChassisSpeeds(drive, xSupplier, ySupplier, omegaSetpoint);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredSpeeds,
            isRed
                ? drive.getRotation().plus(Rotation2d.fromRadians(Math.PI))
                : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    driveYawController.close();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
