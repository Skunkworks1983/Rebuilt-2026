package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.simpleShooter.Shooter;
import frc.robot.subsystems.simpleShooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.NoPitchShooterControls;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedGoal extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController yawController;
  private final Shooter shooter;

  private final LoggedTunableNumber yawKP =
      new LoggedTunableNumber("Auto Aim/Yaw KP", DriveControlConstants.yawControlKp);
  private final LoggedTunableNumber yawKD =
      new LoggedTunableNumber("Auto Aim/Yaw KD", DriveControlConstants.yawControlKD);
  private final LoggedTunableNumber yawTolerance =
      new LoggedTunableNumber("Auto Aim/Yaw Tolerance", DriveControlConstants.yawSetpointTolerance);

  public DriveAimedGoal(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Shooter shooter) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.shooter = shooter;

    yawController =
        new PIDController(
            DriveControlConstants.yawControlKp, 0.0, DriveControlConstants.yawControlKD);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.reset();
    yawController.setP(yawKP.get());
    yawController.setD(yawKD.get());
    yawController.setTolerance(yawTolerance.get());
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds(); // Robot Relative

    NoPitchShooterControls controls =
        ShooterCalculations.getNoPitchExtrapolatedControls(
            currentPose, currentSpeeds, ShooterConstants.shooterPitch);

    shooter.setShooterControls(controls.shooterVelocity(), controls.shooterVelocity());
    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();

    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);

    double omegaCmd = yawController.calculate(currentYawRadians, targetYawRadians);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, xSupplier, ySupplier, omegaCmd // Override rotation
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
    shooter.setShooterControls(RadiansPerSecond.of(0), DegreesPerSecond.of(0));
  }
}
