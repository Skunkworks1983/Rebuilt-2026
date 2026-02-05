package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedGoal {
  /**
   * Uses odometry to continuously aim at the hub and keep flywheel/pivot in correct state to make a
   * shot into the hub Uses the apex constrained solver
   *
   * @param drive The drivetrain subsystem
   * @param shooter The shooter subsystem
   * @param xSupplier Joystick supplier for robot X translation
   * @param ySupplier Joystick supplier for robot Y translation
   */
  public static Command joystickDriveAimedHub(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    PIDController angleController =
        new PIDController(
            DriveControlConstants.yawControlKp, 0.0, DriveControlConstants.yawControlKD);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              ShooterControls controls =
                  ShooterCalculations.getApexConstrainedShooterControls(
                      drive.getPose(), drive.getChassisSpeeds(), drive.getAngularVelocity());
              shooter.setShooterControls(controls);

              Logger.recordOutput(
                  "Auto Shoot/Drive Yaw Setpoint", controls.driveYaw().getRadians());
              double omegaSetpoint =
                  angleController.calculate(
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

              boolean canShoot =
                  shooter.canShoot() && omegaSetpoint < DriveControlConstants.yawSetpointTolerance;
            },
            drive,
            shooter)
        // Reset PID when command initializes to prevent "kick"
        .beforeStarting(() -> angleController.reset())
        .finallyDo(
            () -> {
              angleController.close();
            });
  }
}
