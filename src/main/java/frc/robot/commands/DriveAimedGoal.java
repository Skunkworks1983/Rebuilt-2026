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

public class DriveAimedGoal extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController yawController;

  /**
   * Allows teleop drivetrain translation control while keeping yaw aimed at hub Sets shooter pitch
   * and velocity to correct setpoint for current position
   *
   * @param drive The drivetrain.
   * @param shooter The shooter.
   * @param xSupplier Supplier for control of robot velocity in the x direction of the field frame
   * @param ySupplier Supplier for control of robot velocity in the y direction of the field frame
   */
  public DriveAimedGoal(
      Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    yawController =
        new PIDController(
            DriveControlConstants.YAW_CONTROL_KP, 0.0, DriveControlConstants.YAW_CONTROL_KD);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.reset();
    Logger.recordOutput("Auto Shoot/Drive Aimed Goal", true);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds(); // Robot Relative

    ShooterControls controls =
        ShooterCalculations.getApexConstrainedShooterControls(currentPose, currentSpeeds);

    shooter.setShooterSetpoints(
        new ShooterSetpoints(controls.shooterVelocity(), controls.shooterPitch()));

    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();

    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);

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
    Logger.recordOutput("Auto Shoot/Drive Aimed Goal", false);
  }
}
